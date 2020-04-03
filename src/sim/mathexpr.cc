/*
 * Copyright (c) 2016, 2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "sim/mathexpr.hh"

#include <algorithm>
#include <cmath>
#include <regex>
#include <string>

#include "base/logging.hh"

MathExpr::MathExpr(std::string expr)
 : ops(
     std::array<OpSearch, uNeg + 1> {{
      OpSearch {true, bAdd, 0, '+', [](double a, double b) { return a + b; }},
      OpSearch {true, bSub, 0, '-', [](double a, double b) { return a - b; }},
      OpSearch {true, bMul, 1, '*', [](double a, double b) { return a * b; }},
      OpSearch {true, bDiv, 1, '/', [](double a, double b) { return a / b; }},
      OpSearch {false,uNeg, 2, '-', [](double a, double b) { return -b; }},
      OpSearch {true, bPow, 3, '^', [](double a, double b) {
                 return std::pow(a,b); }
      }},
    })
{
    // Cleanup
    expr.erase(remove_if(expr.begin(), expr.end(), isspace), expr.end());

    root = MathExpr::parse(expr);
    panic_if(!root, "Invalid expression\n");
}

/**
 * This function parses a string expression into an expression tree.
 * It will look for operators in priority order to recursively build the
 * tree, respecting parenthesization.
 * Constants can be expressed in any format accepted by std::stod, whereas
 * variables are essentially [A-Za-z0-9\.$\\]+
 */
MathExpr::Node *
MathExpr::parse(std::string expr) {
    if (expr.size() == 0)
        return NULL;

    // From low to high priority
    int par = 0;
    for (unsigned p = 0; p < MAX_PRIO; p++) {
        for (int i = expr.size() - 1; i >= 0; i--) {
            if (expr[i] == ')')
                par++;
            if (expr[i] == '(')
                par--;

            if (par < 0) return NULL;
            if (par > 0) continue;

            for (unsigned opt = 0; opt < ops.size(); opt++) {
                if (ops[opt].priority != p) continue;
                if (ops[opt].c == expr[i]) {
                    // Try to parse each side
                    Node *l = NULL;
                    if (ops[opt].binary)
                        l = parse(expr.substr(0, i));
                    Node *r = parse(expr.substr(i + 1));
                    if ((l && r) || (!ops[opt].binary && r)) {
                        // Match!
                        Node *n = new Node();
                        n->op = ops[opt].op;
                        n->l = l;
                        n->r = r;
                        return n;
                    }
                }
            }
        }
    }

    // Remove trivial parenthesis
    if (expr.size() >= 2 && expr[0] == '(' && expr[expr.size() - 1] == ')')
        return parse(expr.substr(1, expr.size() - 2));

    // Match a number
    {
        char *sptr;
        double v = strtod(expr.c_str(), &sptr);
        if (sptr != expr.c_str()) {
            Node *n = new Node();
            n->op = sValue;
            n->value = v;
            return n;
        }
    }

    // Match a variable
    {
        bool contains_non_alpha = false;
        for (auto & c: expr)
            contains_non_alpha = contains_non_alpha or
                !( (c >= 'a' && c <= 'z') ||
                   (c >= 'A' && c <= 'Z') ||
                   (c >= '0' && c <= '9') ||
                   c == '$' || c == '\\' || c == '.' || c == '_');

        if (!contains_non_alpha) {
            Node * n = new Node();
            n->op = sVariable;
            n->variable = expr;
            return n;
        }
    }

    return NULL;
}

double
MathExpr::eval(const Node *n, EvalCallback fn) const {
    if (!n)
        return 0;
    else if (n->op == sValue)
        return n->value;
    else if (n->op == sVariable)
        return fn(n->variable);

    for (auto & opt : ops)
        if (opt.op == n->op)
            return opt.fn( eval(n->l, fn), eval(n->r, fn) );

    panic("Invalid node!\n");
    return 0;
}

std::string
MathExpr::toStr(Node *n, std::string prefix) const {
    std::string ret;
    ret += prefix + "|-- " + n->toStr() + "\n";
    if (n->r)
        ret += toStr(n->r, prefix + "|   ");
    if (n->l)
        ret += toStr(n->l, prefix + "|   ");
    return ret;
}

void
MathExpr::getVariables(const Node *n,
                       std::vector<std::string> &variables) const
{
    if (!n || n->op == sValue || n->op == nInvalid) {
        return;
    } else if (n->op == sVariable) {
        variables.push_back(n->variable);
    } else {
        getVariables(n->l, variables);
        getVariables(n->r, variables);
    }
}

