/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef SLEIPNIR_DISABLE_DIAGNOSTICS

#include <stdint.h>

#include <bit>
#include <fstream>
#include <string>
#include <string_view>

#include <Eigen/Sparse>

namespace slp {

/// Writes the sparsity pattern of a sparse matrix to a file.
///
/// Each file represents the sparsity pattern of one matrix over time. <a
/// href="https://github.com/SleipnirGroup/Sleipnir/blob/main/tools/spy.py">spy.py</a>
/// can display it as an animation.
///
/// The file starts with the following header:
/// <ol>
///   <li>Plot title (length as a little-endian int32, then characters)</li>
///   <li>Row label (length as a little-endian int32, then characters)</li>
///   <li>Column label (length as a little-endian int32, then characters)</li>
/// </ol>
///
/// Then, each sparsity pattern starts with:
/// <ol>
///   <li>Number of coordinates as a little-endian int32</li>
/// </ol>
///
/// followed by that many coordinates in the following format:
/// <ol>
///   <li>Row index as a little-endian int32</li>
///   <li>Column index as a little-endian int32</li>
///   <li>Sign as a character ('+' for positive, '-' for negative, or '0' for
///       zero)</li>
/// </ol>
///
/// @tparam Scalar Scalar type.
template <typename Scalar> class Spy {
public:
  /// Constructs a Spy instance.
  ///
  /// @param filename The filename.
  /// @param title Plot title.
  /// @param row_label Row label.
  /// @param col_label Column label.
  /// @param rows The sparse matrix's number of rows.
  /// @param cols The sparse matrix's number of columns.
  Spy(std::string_view filename, std::string_view title,
      std::string_view row_label, std::string_view col_label, int rows,
      int cols)
      : m_file{std::string{filename}, std::ios::binary} {
    // Write title
    write32le(title.size());
    m_file.write(title.data(), title.size());

    // Write row label
    write32le(row_label.size());
    m_file.write(row_label.data(), row_label.size());

    // Write column label
    write32le(col_label.size());
    m_file.write(col_label.data(), col_label.size());

    // Write row and column counts
    write32le(rows);
    write32le(cols);
  }

  /// Adds a matrix to the file.
  ///
  /// @param mat The matrix.
  void add(const Eigen::SparseMatrix<Scalar> &mat) {
    // Write number of coordinates
    write32le(mat.nonZeros());

    // Write coordinates
    for (int k = 0; k < mat.outerSize(); ++k) {
      for (typename Eigen::SparseMatrix<Scalar>::InnerIterator it{mat, k}; it;
           ++it) {
        write32le(it.row());
        write32le(it.col());
        if (it.value() > Scalar(0)) {
          m_file << '+';
        } else if (it.value() < Scalar(0)) {
          m_file << '-';
        } else {
          m_file << '0';
        }
      }
    }
  }

private:
  std::ofstream m_file;

  /// Writes a 32-bit signed integer to the file as little-endian.
  ///
  /// @param num A 32-bit signed integer.
  void write32le(int32_t num) {
    if constexpr (std::endian::native != std::endian::little) {
      // Manual byteswap for 32-bit integer
      num = ((num & 0xFF000000) >> 24) | ((num & 0x00FF0000) >> 8) |
            ((num & 0x0000FF00) << 8) | ((num & 0x000000FF) << 24);
    }
    m_file.write(reinterpret_cast<char *>(&num), sizeof(num));
  }
};

} // namespace slp

#endif
