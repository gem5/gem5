/*
 * Copyright (c) 2012-2014, TU Delft
 * Copyright (c) 2012-2014, TU Eindhoven
 * Copyright (c) 2012-2014, TU Kaiserslautern
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Hansson
 *
 */

#ifndef DATA_PARAMETRISABLE_H
#define DATA_PARAMETRISABLE_H

#include <string>
#include <vector>

#include "Parameter.h"

namespace Data {
/**
 * Convenience class for the architectural components that are
 * parametrisable. The interface is shared and implemented only in
 * this class.
 */
class Parametrisable {
 public:
  Parametrisable()
  {
  }

  virtual ~Parametrisable()
  {
  }

  /**
   * Push a new parameter into the in-order vector without checking
   * for duplicates.
   */
  virtual void pushParameter(const Parameter& parameter);

  /**
   * Set a parameter with a given index (default 0). This could for
   * example be a queue size for a given channel id.
   */
  void setParameter(const Parameter& parameter,
                    unsigned int     index = 0);

  /**
   * Get a parameter of a given name and of a certain index. Calling
   * this method on an object that has no parameter of that name
   * will result in application exit.
   */
  Parameter getParameter(const std::string& id,
                         unsigned int       index = 0) const;

  /**
   * Remove a parameter with a specific name and index. If a parameter
   * is removed this method returns true, otherwise false.
   */
  bool removeParameter(const std::string& id,
                       unsigned int       index = 0);

  /**
   * Simply get all the parameters.
   */
  std::vector<Parameter> getParameters() const;

  /**
   * Check if a parameter of a certain name exists in the object.
   */
  bool hasParameter(const std::string& id,
                    unsigned int       index = 0) const;

  /**
   * Convenience function to set a variable to the value stored in a parameter
   * with built in error detection/assertion. Returns true if the value was
   * successfully set.
   * @param m
   * @param paramName
   * @param assertOnFail
   * @return
   */
  template<typename T>
  bool setVarFromParam(T* m, const char* paramName)
  {
    if (hasParameter(paramName)) {
      *m = static_cast<T>(getParameter(paramName));
      return true;
    } else {
      *m = static_cast<T>(0);
      return false;
    }
  }

  template<typename T>
  T getParamValWithDefault(const char* paramName, T defaultVal) const
  {
    if (hasParameter(paramName)) {
      return static_cast<T>(getParameter(paramName));
    } else   {
      return defaultVal;
    }
  }

 protected:
  std::vector<Parameter> parameters;
};
}
#endif // ifndef DATA_PARAMETRISABLE_H
