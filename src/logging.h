/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#pragma once
#include <etl/cstring.h>
// #include <fmt/core.h>
// #include <fmt/format.h>
#include <gsl/gsl>

namespace logging {

/// Line size including tick number and task name.
static constexpr size_t MAX_LINE_SIZE = 256;

/// Have to be initialized.
void init ();

/**
 * Add a log. Do not call from an ISR.
 */
bool log (gsl::czstring<> str);

// template <typename Format, typename... Args> bool log2 (Format &&format, Args &&... args)
// {
//         etl::string<MAX_LINE_SIZE> buf;
//         fmt::format_to (std::back_inserter (buf), std::forward<Format> (format), std::forward<Args> (args)...);
//         log (buf.c_str ());
//         return true;
// }

} // namespace logging