#include "include_ColmapDatabase.hpp"

#include <cstddef>
#include <cstring>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <sqlite3.h>

namespace {

std::string SqliteError(sqlite3* db, const std::string& prefix) {
  const char* message = db != nullptr ? sqlite3_errmsg(db) : "sqlite error";
  return prefix + ": " + message;
}

void CheckImageId(int image_id) {
  if (image_id <= 0) {
    throw std::invalid_argument("image_id doit etre strictement positif.");
  }
}

}  // namespace

namespace loc {

ColmapDatabase::ColmapDatabase(const std::string& database_path)
    : database_path_(database_path) {
  if (database_path_.empty()) {
    throw std::invalid_argument("Le chemin vers la base COLMAP est vide.");
  }

  const int rc = sqlite3_open_v2(
      database_path_.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr);
  if (rc != SQLITE_OK) {
    std::string message = SqliteError(db_, "Impossible d'ouvrir la base " + database_path_);
    if (db_ != nullptr) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
    throw std::runtime_error(message);
  }
}

ColmapDatabase::~ColmapDatabase() {
  if (db_ != nullptr) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
}

ColmapDatabase::ColmapDatabase(ColmapDatabase&& other) noexcept
    : db_(other.db_), database_path_(std::move(other.database_path_)) {
  other.db_ = nullptr;
}

ColmapDatabase& ColmapDatabase::operator=(ColmapDatabase&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  if (db_ != nullptr) {
    sqlite3_close(db_);
  }

  db_ = other.db_;
  database_path_ = std::move(other.database_path_);
  other.db_ = nullptr;
  return *this;
}

cv::Mat ColmapDatabase::LoadKeypoints(int image_id) const {
  CheckImageId(image_id);
  if (db_ == nullptr) {
    throw std::runtime_error("Base COLMAP non initialisee.");
  }

  const std::string sql = "SELECT rows, cols, data FROM keypoints WHERE image_id = ?1;";

  sqlite3_stmt* statement = nullptr;
  const int prepare_rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &statement, nullptr);
  if (prepare_rc != SQLITE_OK) {
    throw std::runtime_error(SqliteError(db_, "Echec de preparation de la requete sur keypoints"));
  }

  const auto finalize_statement = [&statement]() {
    if (statement != nullptr) {
      sqlite3_finalize(statement);
      statement = nullptr;
    }
  };

  try {
    if (sqlite3_bind_int(statement, 1, image_id) != SQLITE_OK) {
      throw std::runtime_error(SqliteError(db_, "Echec du bind image_id sur keypoints"));
    }

    const int step_rc = sqlite3_step(statement);
    if (step_rc == SQLITE_DONE) {
      throw std::runtime_error("Aucune entree keypoints pour image_id=" + std::to_string(image_id) + ".");
    }
    if (step_rc != SQLITE_ROW) {
      throw std::runtime_error(SqliteError(db_, "Echec de lecture sur keypoints"));
    }

    const int rows = sqlite3_column_int(statement, 0);
    const int cols = sqlite3_column_int(statement, 1);
    const void* blob = sqlite3_column_blob(statement, 2);
    const int blob_size = sqlite3_column_bytes(statement, 2);

    if (rows < 0 || cols < 0) {
      throw std::runtime_error("Dimensions negatives dans keypoints pour image_id=" +
                               std::to_string(image_id) + ".");
    }

    if (cols != 2 && cols != 4 && cols != 6) {
      throw std::runtime_error("Nombre de colonnes keypoints non supporte pour image_id=" +
                               std::to_string(image_id) +
                               ": attendu 2, 4 ou 6; lu=" + std::to_string(cols) + ".");
    }

    const std::size_t rows_sz = static_cast<std::size_t>(rows);
    const std::size_t cols_sz = static_cast<std::size_t>(cols);
    if (rows_sz > 0 && cols_sz > std::numeric_limits<std::size_t>::max() / rows_sz) {
      throw std::runtime_error("Overflow dimensions dans keypoints.");
    }

    const std::size_t num_values = rows_sz * cols_sz;
    if (num_values > 0 && sizeof(float) > std::numeric_limits<std::size_t>::max() / num_values) {
      throw std::runtime_error("Overflow taille BLOB dans keypoints.");
    }

    const std::size_t expected_blob_size = num_values * sizeof(float);
    if (expected_blob_size != static_cast<std::size_t>(blob_size)) {
      throw std::runtime_error("Taille BLOB invalide dans keypoints pour image_id=" +
                               std::to_string(image_id) +
                               ": attendu=" + std::to_string(expected_blob_size) +
                               ", lu=" + std::to_string(blob_size) + ".");
    }

    if (expected_blob_size > 0 && blob == nullptr) {
      throw std::runtime_error("BLOB nul dans keypoints pour image_id=" +
                               std::to_string(image_id) + ".");
    }

    cv::Mat result(rows, 4, CV_32F, cv::Scalar(0.0F));
    if (rows > 0) {
      const float* src = static_cast<const float*>(blob);
      for (int r = 0; r < rows; ++r) {
        const float* row_src = src + static_cast<std::size_t>(r) * static_cast<std::size_t>(cols);
        float* row_dst = result.ptr<float>(r);
        row_dst[0] = row_src[0];
        row_dst[1] = row_src[1];
        if (cols == 4) {
          row_dst[2] = row_src[2];
          row_dst[3] = row_src[3];
        } else {
          row_dst[2] = 1.0F;
          row_dst[3] = 0.0F;
        }
      }
    }

    const int second_step_rc = sqlite3_step(statement);
    if (second_step_rc != SQLITE_DONE) {
      throw std::runtime_error("Plusieurs lignes inattendues dans keypoints pour image_id=" +
                               std::to_string(image_id) + ".");
    }

    finalize_statement();
    return result;
  } catch (...) {
    finalize_statement();
    throw;
  }
}

cv::Mat ColmapDatabase::LoadDescriptors(int image_id) const {
  return LoadMatrixBlob(image_id, "descriptors", 128, CV_8U, static_cast<int>(sizeof(std::uint8_t)));
}

cv::Mat ColmapDatabase::LoadMatrixBlob(int image_id,
                                       const char* table_name,
                                       int expected_cols,
                                       int cv_type,
                                       int element_size) const {
  CheckImageId(image_id);
  if (db_ == nullptr) {
    throw std::runtime_error("Base COLMAP non initialisee.");
  }

  const std::string sql =
      "SELECT rows, cols, data FROM " + std::string(table_name) + " WHERE image_id = ?1;";

  sqlite3_stmt* statement = nullptr;
  const int prepare_rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &statement, nullptr);
  if (prepare_rc != SQLITE_OK) {
    throw std::runtime_error(SqliteError(db_, "Echec de preparation de la requete sur " + std::string(table_name)));
  }

  const auto finalize_statement = [&statement]() {
    if (statement != nullptr) {
      sqlite3_finalize(statement);
      statement = nullptr;
    }
  };

  try {
    if (sqlite3_bind_int(statement, 1, image_id) != SQLITE_OK) {
      throw std::runtime_error(SqliteError(db_, "Echec du bind image_id sur " + std::string(table_name)));
    }

    const int step_rc = sqlite3_step(statement);
    if (step_rc == SQLITE_DONE) {
      throw std::runtime_error("Aucune entree " + std::string(table_name) +
                               " pour image_id=" + std::to_string(image_id) + ".");
    }
    if (step_rc != SQLITE_ROW) {
      throw std::runtime_error(SqliteError(db_, "Echec de lecture sur " + std::string(table_name)));
    }

    const int rows = sqlite3_column_int(statement, 0);
    const int cols = sqlite3_column_int(statement, 1);
    const void* blob = sqlite3_column_blob(statement, 2);
    const int blob_size = sqlite3_column_bytes(statement, 2);

    if (rows < 0 || cols < 0) {
      throw std::runtime_error("Dimensions negatives dans " + std::string(table_name) +
                               " pour image_id=" + std::to_string(image_id) + ".");
    }
    if (cols != expected_cols) {
      throw std::runtime_error("Nombre de colonnes invalide dans " + std::string(table_name) +
                               " pour image_id=" + std::to_string(image_id) +
                               ": attendu=" + std::to_string(expected_cols) +
                               ", lu=" + std::to_string(cols) + ".");
    }

    const std::size_t rows_sz = static_cast<std::size_t>(rows);
    const std::size_t cols_sz = static_cast<std::size_t>(cols);
    if (rows_sz > 0 && cols_sz > std::numeric_limits<std::size_t>::max() / rows_sz) {
      throw std::runtime_error("Overflow dimensions dans " + std::string(table_name) + ".");
    }

    const std::size_t num_values = rows_sz * cols_sz;
    if (num_values > 0 && static_cast<std::size_t>(element_size) > std::numeric_limits<std::size_t>::max() / num_values) {
      throw std::runtime_error("Overflow taille BLOB dans " + std::string(table_name) + ".");
    }

    const std::size_t expected_blob_size = num_values * static_cast<std::size_t>(element_size);
    if (expected_blob_size != static_cast<std::size_t>(blob_size)) {
      throw std::runtime_error("Taille BLOB invalide dans " + std::string(table_name) +
                               " pour image_id=" + std::to_string(image_id) +
                               ": attendu=" + std::to_string(expected_blob_size) +
                               ", lu=" + std::to_string(blob_size) + ".");
    }

    if (expected_blob_size > 0 && blob == nullptr) {
      throw std::runtime_error("BLOB nul dans " + std::string(table_name) +
                               " pour image_id=" + std::to_string(image_id) + ".");
    }

    cv::Mat matrix(rows, cols, cv_type);
    if (expected_blob_size > 0) {
      std::memcpy(matrix.data, blob, expected_blob_size);
    }

    const int second_step_rc = sqlite3_step(statement);
    if (second_step_rc != SQLITE_DONE) {
      throw std::runtime_error("Plusieurs lignes inattendues dans " + std::string(table_name) +
                               " pour image_id=" + std::to_string(image_id) + ".");
    }

    finalize_statement();
    return matrix;
  } catch (...) {
    finalize_statement();
    throw;
  }
}

}  // namespace loc