// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

// From
// https://gitlab.com/libeigen/eigen/-/raw/d78a97506a159036b824a8a78cbf13be5716ecd2/unsupported/Eigen/Format
// Eigen should accept this PR :)

#ifndef CAMERA_CALIBRATION_SRC_EIGENFORMAT_HPP_
#define CAMERA_CALIBRATION_SRC_EIGENFORMAT_HPP_

#include <Eigen/Core>
#include <Eigen/SparseCore>

#if __cplusplus >= 202002

#include <concepts>
#include <format>

/**
 * Formatter for classes derived from Eigen::DenseBase<Derived> or
 * Eigen::SparseCompressedBase<Derived>.
 */
template <typename Derived, typename CharT>
  requires std::derived_from<Derived, Eigen::DenseBase<Derived>> ||
           std::derived_from<Derived, Eigen::SparseCompressedBase<Derived>>
struct std::formatter<Derived, CharT> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    return m_underlying.parse(ctx);
  }

  template <typename FmtContext>
  auto format(const Derived &mat, FmtContext &ctx) const {
    auto out = ctx.out();

    for (Eigen::Index row = 0; row < mat.rows(); ++row) {
      for (Eigen::Index col = 0; col < mat.cols(); ++col) {
        out = std::format_to(out, "  ");
        out = m_underlying.format(mat.coeff(row, col), ctx);
      }

      if (row < mat.rows() - 1) {
        out = std::format_to(out, "\n");
      }
    }

    return out;
  }

private:
  std::formatter<typename Derived::Scalar, CharT> m_underlying;
};

#endif // __cpp_lib_format >= 201907L

#endif // CAMERA_CALIBRATION_SRC_EIGENFORMAT_HPP_
