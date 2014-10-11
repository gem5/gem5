#ifndef __LIBUTIL_CALCULATOR_H__
#define __LIBUTIL_CALCULATOR_H__

#include <sstream>

#include "String.h"
#include "Map.h"
#include "Assert.h"

namespace LibUtil
{
    using std::istringstream;

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
            void evaluateString(const String& str_);

        protected:
            Token getToken(istringstream& ist_);
            double prim(istringstream& ist_, bool is_get_);
            double term(istringstream& ist_, bool is_get_);
            double expr(istringstream& ist_, bool is_get_);
            virtual double getEnvVar(const String& var_name_) const;

        protected:
            String m_reserved_chars_;
            Map<double> m_var_;

            Token m_curr_token_;
            double m_value_number_;
            String m_value_string_;
    }; // class Calculator
} // namespace LibUtil

#endif // __LIBUTIL_CALCULATOR_H__

