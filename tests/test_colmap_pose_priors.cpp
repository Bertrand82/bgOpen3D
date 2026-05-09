#include "colmap/pose_prior.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstring>

#include <sqlite3.h>

namespace {

std::filesystem::path FindDatabasePath() {
  const std::vector<std::filesystem::path> candidates = {
      "data/database.db",
      "../data/database.db",
      "../../data/database.db",
      "../../../data/database.db",
  };

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error("Impossible de trouver data/database.db");
}

// Deserialise un BLOB SQLite de N doubles dans un tableau Eigen contigu.
template <typename EigenType>
EigenType BlobToEigen(sqlite3_stmt* stmt, int col) {
  const void* ptr = sqlite3_column_blob(stmt, col);
  const int bytes = sqlite3_column_bytes(stmt, col);
  EigenType result = EigenType::Constant(std::numeric_limits<double>::quiet_NaN());
  if (ptr != nullptr && bytes == static_cast<int>(sizeof(double) * EigenType::SizeAtCompileTime)) {
    std::memcpy(result.data(), ptr, static_cast<std::size_t>(bytes));
  }
  return result;
}

colmap::PosePrior ReadPosePriorFromRow(sqlite3_stmt* stmt) {
  colmap::PosePrior prior;

  // Colonnes: pose_prior_id, corr_data_id, corr_sensor_id, corr_sensor_type,
  //           position, position_covariance, gravity, coordinate_system
  prior.pose_prior_id =
      static_cast<colmap::pose_prior_t>(sqlite3_column_int64(stmt, 0));

  const auto sensor_type =
      static_cast<colmap::SensorType>(sqlite3_column_int(stmt, 3));
  const auto sensor_id =
      static_cast<uint32_t>(sqlite3_column_int64(stmt, 2));
  const auto data_id =
      static_cast<uint32_t>(sqlite3_column_int64(stmt, 1));
  prior.corr_data_id = colmap::data_t(colmap::sensor_t(sensor_type, sensor_id), data_id);

  prior.position = BlobToEigen<Eigen::Vector3d>(stmt, 4);

  const void* cov_ptr = sqlite3_column_blob(stmt, 5);
  const int cov_bytes = sqlite3_column_bytes(stmt, 5);
  if (cov_ptr != nullptr && cov_bytes == static_cast<int>(9 * sizeof(double))) {
    std::memcpy(prior.position_covariance.data(), cov_ptr,
                static_cast<std::size_t>(cov_bytes));
  } else {
    prior.position_covariance =
        Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }

  prior.gravity = BlobToEigen<Eigen::Vector3d>(stmt, 6);

  prior.coordinate_system = static_cast<colmap::PosePrior::CoordinateSystem>(
      sqlite3_column_int(stmt, 7));

  return prior;
}

void PrintPosePrior(const colmap::PosePrior& prior) {
  std::cout << "  pose_prior_id     : " << prior.pose_prior_id << "\n";
  std::cout << "  corr_sensor_type  : " << prior.corr_data_id.sensor_id.type << "\n";
  std::cout << "  corr_sensor_id    : " << prior.corr_data_id.sensor_id.id << "\n";
  std::cout << "  corr_data_id      : " << prior.corr_data_id.id << "\n";
  std::cout << "  coordinate_system : " << static_cast<int>(prior.coordinate_system) << "\n";
  if (prior.HasPosition()) {
    std::cout << "  position          : ["
              << prior.position.x() << ", "
              << prior.position.y() << ", "
              << prior.position.z() << "]\n";
  } else {
    std::cout << "  position          : (non definie)\n";
  }
  if (prior.HasPositionCov()) {
    std::cout << "  position_cov diag : ["
              << prior.position_covariance(0, 0) << ", "
              << prior.position_covariance(1, 1) << ", "
              << prior.position_covariance(2, 2) << "]\n";
  } else {
    std::cout << "  position_cov      : (non definie)\n";
  }
  if (prior.HasGravity()) {
    std::cout << "  gravity           : ["
              << prior.gravity.x() << ", "
              << prior.gravity.y() << ", "
              << prior.gravity.z() << "]\n";
  } else {
    std::cout << "  gravity           : (non definie)\n";
  }
}

}  // namespace

int main() {
  try {
    const std::filesystem::path db_path = FindDatabasePath();
    std::cout << "Ouverture base COLMAP: " << db_path << "\n";

    sqlite3* db = nullptr;
    const int open_rc = sqlite3_open_v2(db_path.string().c_str(), &db,
                                        SQLITE_OPEN_READONLY, nullptr);
    if (open_rc != SQLITE_OK) {
      throw std::runtime_error("Impossible d'ouvrir la base de donnees");
    }

    const auto close_db = [&db]() {
      if (db != nullptr) {
        sqlite3_close(db);
        db = nullptr;
      }
    };

    try {
      const char* sql =
          "SELECT pose_prior_id, corr_data_id, corr_sensor_id, corr_sensor_type,"
          "       position, position_covariance, gravity, coordinate_system"
          " FROM pose_priors;";
      sqlite3_stmt* stmt = nullptr;

      const int prepare_rc = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
      if (prepare_rc != SQLITE_OK) {
        const char* err = sqlite3_errmsg(db);
        if (std::string(err).find("no such table") != std::string::npos) {
          std::cout << "Table 'pose_priors' non trouvee dans la base.\n";
          close_db();
          return 0;
        }
        throw std::runtime_error(std::string("Echec preparation requete: ") + err);
      }

      const auto finalize_stmt = [&stmt]() {
        if (stmt != nullptr) {
          sqlite3_finalize(stmt);
          stmt = nullptr;
        }
      };

      try {
        std::vector<colmap::PosePrior> priors;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
          priors.push_back(ReadPosePriorFromRow(stmt));
        }
        finalize_stmt();

        std::cout << "Nombre de PosePrior lus: " << priors.size() << "\n";
        if (priors.empty()) {
          std::cout << "Aucune entree pose_priors dans la base.\n";
        }
        for (std::size_t i = 0; i < priors.size(); ++i) {
          std::cout << "\n>>> PosePrior " << (i + 1) << "\n";
          PrintPosePrior(priors[i]);
        }

        std::cout << "\nTest pose_priors OK\n";
        close_db();
        return 0;
      } catch (...) {
        finalize_stmt();
        throw;
      }
    } catch (...) {
      close_db();
      throw;
    }
  } catch (const std::exception& e) {
    std::cerr << "Test pose_priors KO: " << e.what() << "\n";
    return 1;
  }
}
