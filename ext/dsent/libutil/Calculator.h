/* Copyright (c) 2012 Massachusetts Institute of Technology
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __LIBUTIL_CALCULATOR_H__
#define __LIBUTIL_CALCULATOR_H__

#include <sstream>

#include "model/Model.h"
#include "String.h"
#include "Map.h"
#include "Assert.h"

namespace LibUtil
{
    using std::istringstream;
    using std::ostringstream;

    /*
     *  program:
     *      END                         // END is end-of-input
     *      expr_list END
     *
     *  expr_list:
     *      expression SEP expr_list    // SEP is semicolon
     *      expression                  
     *      print expression
     *      print STRING
     *      print STRING expression
     *      print STRING expression SEP expr_list
     *
     *
     *  expression:
     *      expression + term
     *      expression - term
     *      term
     *
     *  term:
     *      term / primary
     *      term * primary
     *      primary
     *
     *  primary:
     *      NUMBER
     *      NAME
     *      NAME = expression
     *      NAME string expression      // NAME is print
     *      - primary
     *      ( expression )
     *
     *  string:
     *      
     **/

    class Calculator
    {
        protected:
            enum Token
            {
                NAME, NAME2, NUMBER, STRING, END,
                PLUS = '+', MINUS = '-', MUL = '*', DIV = '/',
                SEP = ';', ASSIGN = '=', LP = '(', RP = ')'
            };

        public:
            Calculator();
            virtual ~Calculator();

        public:
            void reset();
            void evaluateString(const String& str_,
                                const std::map<String, String> &config,
                                DSENT::Model *ms_model,
                                std::map<std::string, double> &outputs);

        protected:
            Token getToken(istringstream& ist_);

            double prim(istringstream& ist_, bool is_get_,
                        const std::map<String, String> &config,
                        DSENT::Model *ms_model);

            double term(istringstream& ist_, bool is_get_,
                        const std::map<String, String> &config,
                        DSENT::Model *ms_model);

            double expr(istringstream& ist_, bool is_get_,
                        const std::map<String, String> &config,
                        DSENT::Model *ms_model);

            virtual double getEnvVar(
                    const String& var_name_,
                    const std::map<String, String> &config,
                    DSENT::Model *ms_model) const;

        protected:
            String m_reserved_chars_;
            Map<double> m_var_;

            Token m_curr_token_;
            double m_value_number_;
            String m_value_string_;
    };
} // namespace LibUtil

#endif // __LIBUTIL_CALCULATOR_H__

