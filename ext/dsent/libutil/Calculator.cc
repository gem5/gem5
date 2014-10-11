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

#include "Calculator.h"

#include <cctype>
#include <iostream>

namespace LibUtil
{
    using std::cout;
    using std::endl;
    using std::scientific;

    Calculator::Calculator()
    {
        m_reserved_chars_ = "+-*/;=()\\";
    }

    Calculator::~Calculator()
    {}

    void Calculator::reset()
    {
        m_var_.clear();
        return;
    }

    void Calculator::evaluateString(const String& str_,
                                    const map<String, String> &config,
                                    DSENT::Model *ms_model,
                                    map<string, double> &outputs)
    {
        istringstream ist(str_);

        while(ist)
        {
            getToken(ist);
            if(m_curr_token_ == END) break;
            if(m_curr_token_ == SEP) continue;
            if((m_curr_token_ == NAME) && (m_value_string_ == "print"))
            {
                getToken(ist);

                if(m_curr_token_ == STRING)
                {
                    String print_str = m_value_string_;

                    getToken(ist);
                    if(m_curr_token_ == SEP)
                    {
                        outputs[print_str] = 0;
                        cout << print_str << endl;
                    }
                    else
                    {
                        double v = expr(ist, false, config, ms_model);
                        outputs[print_str] = v;
                        cout << print_str << v << endl;
                    }
                }
                else
                {
                    double v = expr(ist, false, config, ms_model);
                    outputs["Missing Expression"] = v;
                    cout << v << endl;
                }
            }
            else
            {
                expr(ist, false, config, ms_model);
            }
        }
    }

    Calculator::Token Calculator::getToken(istringstream& ist_)
    {
        char ch;
        do
        {
            ist_.get(ch);
            if(!ist_)
            {
                m_curr_token_ = END;
                return m_curr_token_;
            }
        }
        while(ch != '\n' && isspace(ch));

        switch(ch)
        {
            case '\n':
                m_curr_token_ = END;
                return m_curr_token_;
            case ';':
                m_curr_token_ = SEP;
                return m_curr_token_;
            case '*':
            case '/':
            case '+':
            case '-':
            case '(':
            case ')':
            case '=':
                m_curr_token_ = Token(ch);
                return m_curr_token_;
            case '0': case '1': case '2': case '3': case '4':
            case '5': case '6': case '7': case '8': case '9':
            case '.':
                ist_.putback(ch);
                ist_ >> m_value_number_;
                m_curr_token_ = NUMBER;
                return m_curr_token_;
            case '"':
                ist_.get(ch);
                m_value_string_ = "";
                while(ist_ && ('"' != ch))
                {
                    m_value_string_ += String(1, ch);
                    ist_.get(ch);
                }
                m_curr_token_ = STRING;
                return m_curr_token_;
            case '$':
                ist_.get(ch);
                ASSERT((ch == '('), "[Error] Bad token: '(' expected");
                ist_.get(ch);
                m_value_string_ = "";
                while(ist_ && (!isspace(ch)) && (')' != ch))
                {
                    m_value_string_ += String(1, ch);
                    ist_.get(ch);
                }
                m_curr_token_ = NAME2;
                return m_curr_token_;
            default:
                if(isalpha(ch))
                {
                    m_value_string_ = ch;
                    ist_.get(ch);
                    while(ist_ && (isalnum(ch) || ('_' == ch)))
                    {
                        m_value_string_ += String(1, ch);
                        ist_.get(ch);
                    }
                    ist_.putback(ch);
                    m_curr_token_ = NAME;
                    return m_curr_token_;
                }
                else
                {
                    String error_msg = "[Error] Bad token: '" + String(ch) + "'";
                    throw Exception(error_msg);
                }
        }
    }

    double Calculator::prim(istringstream& ist_, bool is_get_,
                            const map<String, String> &config,
                            DSENT::Model *ms_model)
    {
        if(is_get_)
        {
            getToken(ist_);
        }

        double v;
        switch(m_curr_token_)
        {
            case NUMBER:
                v = m_value_number_;
                getToken(ist_);
                return v;
            case NAME:
                if(getToken(ist_) == ASSIGN)
                {
                    String var_name = m_value_string_;
                    v = expr(ist_, true, config, ms_model);
                    m_var_.set(var_name, v);
                }
                else
                {
                    v = m_var_.get(m_value_string_);
                }
                return v;
            case NAME2:
                v = getEnvVar(m_value_string_, config, ms_model);
                getToken(ist_);
                return v;
            case MINUS:
                return -prim(ist_, true, config, ms_model);
            case LP:
                v = expr(ist_, true, config, ms_model);
                ASSERT((m_curr_token_ == RP), "[Error] ')' expected");
                getToken(ist_);
                return v;
            default:
                ASSERT(0, "[Error] primary expected, get: '" + String(int(m_curr_token_)) + "'");
        }
    }

    double Calculator::term(istringstream& ist_, bool is_get_,
                            const map<String, String> &config,
                            DSENT::Model *ms_model)
    {
        double left = prim(ist_, is_get_, config, ms_model);

        while(1)
        {
            double d;
            switch(m_curr_token_)
            {
                case MUL:
                    left *= prim(ist_, true, config, ms_model);
                    break;
                case DIV:
                    d = prim(ist_, true, config, ms_model);
                    ASSERT(d, "[Error] divided by 0");
                    left /= d;
                    break;
                default:
                    return left;
            }
        }
    }

    double Calculator::expr(istringstream& ist_, bool is_get_,
                            const map<String, String> &config,
                            DSENT::Model *ms_model)
    {
        double left = term(ist_, is_get_, config, ms_model);

        while(1)
        {
            switch(m_curr_token_)
            {
                case PLUS:
                    left += term(ist_, true, config, ms_model);
                    break;
                case MINUS:
                    left -= term(ist_, true, config, ms_model);
                    break;
                default:
                    return left;
            }
        }
    }

    double Calculator::getEnvVar(const String& var_name_,
                                 const map<String, String> &config,
                                 DSENT::Model *ms_model) const
    {
        return m_var_.get(var_name_);
    }
} // namespace LibUtil
