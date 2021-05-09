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

#ifndef __SIM_MATHEXPR_HH__
#define __SIM_MATHEXPR_HH__

#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <vector>

namespace gem5
{

class MathExpr
{
  public:

    MathExpr(std::string expr);

    typedef std::function<double(std::string)> EvalCallback;

    /**
     * Prints an ASCII representation of the expression tree
     *
     * @return A string containing the ASCII representation of the expression
     */
    std::string toStr() const { return toStr(root, ""); }

    /**
     * Evaluates the expression
     *
     * @param fn A callback funcion to evaluate variables
     *
     * @return The value for this expression
     */
    double eval(EvalCallback fn) const { return eval(root, fn); }

    /**
     * Return all variables in the this expression.
     *
     * This function starts from the root node and traverses all nodes
     * while adding the variables it finds to a vector. Returns the
     * found variables in a vector of strings
     *
     * @return A Vector with the names of all variables
    */
    std::vector<std::string> getVariables() const
    {
        std::vector<std::string> vars;
        getVariables(root, vars);
        return vars;
    }

  private:
    enum Operator
    {
        bAdd, bSub, bMul, bDiv, bPow, uNeg, sValue, sVariable, nInvalid
    };

    // Match operators
    const int MAX_PRIO = 4;
    typedef double (*binOp)(double, double);
    struct OpSearch
    {
        bool binary;
        Operator op;
        int priority;
        char c;
        binOp fn;
    };

    /** Operator list */
    std::array<OpSearch, uNeg + 1> ops;

    class Node
    {
      public:
        Node() : op(nInvalid), l(0), r(0), value(0) {}
        std::string toStr() const {
            const char opStr[] = {'+', '-', '*', '/', '^', '-'};
            switch (op) {
              case nInvalid:
                return "INVALID";
              case sVariable:
                return variable;
              case sValue:
                return std::to_string(value);
              default:
                return std::string(1, opStr[op]);
            };
        }

        Operator op;
        Node *l, *r;
        double value;
        std::string variable;
    };

    /** Root node */
    Node * root;

    /** Parse and create nodes from string */
    Node *parse(std::string expr);

    /** Print tree as string */
    std::string toStr(Node *n, std::string prefix) const;

    /** Eval a node */
    double eval(const Node *n, EvalCallback fn) const;

    /** Return all variable reachable from a node to a vector of
     * strings */
    void getVariables(const Node *n, std::vector<std::string> &vars) const;
};

} // namespace gem5

#endif
