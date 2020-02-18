/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#pragma once
#include <gsl/gsl>

namespace logging {

void init ();

/**
 * Add a log. Do not call from an ISR.
 */
bool log (gsl::czstring<> str);

} // namespace logging