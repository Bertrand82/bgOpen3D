#pragma once

#include <string>

#include <opencv2/core.hpp>

struct sqlite3;

namespace loc {

class ColmapDatabase {
public:
  explicit ColmapDatabase(const std::string& database_path);
  ~ColmapDatabase();

  ColmapDatabase(const ColmapDatabase&) = delete;
  ColmapDatabase& operator=(const ColmapDatabase&) = delete;

  ColmapDatabase(ColmapDatabase&& other) noexcept;
  ColmapDatabase& operator=(ColmapDatabase&& other) noexcept;

  cv::Mat LoadKeypoints(int image_id) const;
  cv::Mat LoadDescriptors(int image_id) const;

private:
  cv::Mat LoadMatrixBlob(int image_id,
                         const char* table_name,
                         int expected_cols,
                         int cv_type,
                         int element_size) const;

  sqlite3* db_ = nullptr;
  std::string database_path_;
};

}  // namespace loc