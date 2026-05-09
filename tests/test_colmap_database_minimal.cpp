#include "include_ColmapDatabase.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <sqlite3.h>

namespace {

class SqliteOwnedDb {
public:
  explicit SqliteOwnedDb(const std::filesystem::path& path) {
    const int rc = sqlite3_open(path.string().c_str(), &db_);
    if (rc != SQLITE_OK) {
      throw std::runtime_error("Impossible de creer la base de test.");
    }
  }

  ~SqliteOwnedDb() {
    if (db_ != nullptr) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

  sqlite3* get() const { return db_; }

private:
  sqlite3* db_ = nullptr;
};

void Exec(sqlite3* db, const char* sql) {
  char* error_message = nullptr;
  const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &error_message);
  if (rc != SQLITE_OK) {
    const std::string error = error_message != nullptr ? error_message : "sqlite exec error";
    sqlite3_free(error_message);
    throw std::runtime_error(error);
  }
}

void InsertBlob(sqlite3* db,
                const char* table_name,
                int image_id,
                int rows,
                int cols,
                const void* data,
                int size_bytes) {
  const std::string sql =
      "INSERT INTO " + std::string(table_name) + "(image_id, rows, cols, data) VALUES(?1, ?2, ?3, ?4);";

  sqlite3_stmt* statement = nullptr;
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &statement, nullptr) != SQLITE_OK) {
    throw std::runtime_error("Preparation INSERT impossible.");
  }

  const auto finalize_statement = [&statement]() {
    if (statement != nullptr) {
      sqlite3_finalize(statement);
      statement = nullptr;
    }
  };

  try {
    sqlite3_bind_int(statement, 1, image_id);
    sqlite3_bind_int(statement, 2, rows);
    sqlite3_bind_int(statement, 3, cols);
    sqlite3_bind_blob(statement, 4, data, size_bytes, SQLITE_STATIC);

    if (sqlite3_step(statement) != SQLITE_DONE) {
      throw std::runtime_error("Insertion BLOB impossible.");
    }

    finalize_statement();
  } catch (...) {
    finalize_statement();
    throw;
  }
}

bool AlmostEqual(float a, float b, float eps = 1e-6F) {
  return std::fabs(a - b) <= eps;
}

}  // namespace

int main() {
  const std::filesystem::path db_path =
      std::filesystem::temp_directory_path() / "georeloc_test_colmap_database_minimal.db";

  try {
    std::filesystem::remove(db_path);

    SqliteOwnedDb sqlite_db(db_path);
    Exec(sqlite_db.get(),
         "CREATE TABLE keypoints(image_id INTEGER PRIMARY KEY, rows INTEGER NOT NULL, cols INTEGER NOT NULL, data BLOB);"
         "CREATE TABLE descriptors(image_id INTEGER PRIMARY KEY, rows INTEGER NOT NULL, cols INTEGER NOT NULL, data BLOB);");

    const std::array<float, 8> keypoints = {
        10.5F, 20.25F, 1.5F, 0.75F,
        30.0F, 40.5F, 2.0F, -1.25F,
    };

    std::vector<std::uint8_t> descriptors(2U * 128U);
    for (std::size_t i = 0; i < descriptors.size(); ++i) {
      descriptors[i] = static_cast<std::uint8_t>(i % 251U);
    }

    InsertBlob(sqlite_db.get(),
               "keypoints",
               1,
               2,
               4,
               keypoints.data(),
               static_cast<int>(sizeof(keypoints)));
    InsertBlob(sqlite_db.get(),
               "descriptors",
               1,
               2,
               128,
               descriptors.data(),
               static_cast<int>(descriptors.size()));

    loc::ColmapDatabase database(db_path.string());

    const cv::Mat loaded_keypoints = database.LoadKeypoints(1);
    if (loaded_keypoints.type() != CV_32F) {
      throw std::runtime_error("Type keypoints attendu=CV_32F.");
    }
    if (loaded_keypoints.rows != 2 || loaded_keypoints.cols != 4) {
      throw std::runtime_error("Dimensions keypoints attendues=2x4.");
    }
    if (!AlmostEqual(loaded_keypoints.at<float>(0, 0), 10.5F) ||
        !AlmostEqual(loaded_keypoints.at<float>(0, 3), 0.75F) ||
        !AlmostEqual(loaded_keypoints.at<float>(1, 1), 40.5F) ||
        !AlmostEqual(loaded_keypoints.at<float>(1, 3), -1.25F)) {
      throw std::runtime_error("Valeurs keypoints incorrectes.");
    }

    const cv::Mat loaded_descriptors = database.LoadDescriptors(1);
    if (loaded_descriptors.type() != CV_8U) {
      throw std::runtime_error("Type descriptors attendu=CV_8U.");
    }
    if (loaded_descriptors.rows != 2 || loaded_descriptors.cols != 128) {
      throw std::runtime_error("Dimensions descriptors attendues=2x128.");
    }
    if (loaded_descriptors.at<std::uint8_t>(0, 0) != descriptors[0] ||
        loaded_descriptors.at<std::uint8_t>(0, 127) != descriptors[127] ||
        loaded_descriptors.at<std::uint8_t>(1, 0) != descriptors[128] ||
        loaded_descriptors.at<std::uint8_t>(1, 127) != descriptors[255]) {
      throw std::runtime_error("Valeurs descriptors incorrectes.");
    }

    std::filesystem::remove(db_path);
    return 0;
  } catch (const std::exception& e) {
    std::filesystem::remove(db_path);
    std::cerr << "Test ColmapDatabase minimal KO: " << e.what() << std::endl;
    return 1;
  }
}