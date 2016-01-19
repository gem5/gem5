/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marc Orr
 */

#include <csignal>

#include "arch/hsail/insts/decl.hh"
#include "arch/hsail/insts/mem.hh"

namespace HsailISA
{
    // Pseudo (or magic) instructions are overloaded on the hsail call
    // instruction, because of its flexible parameter signature.

    // To add a new magic instruction:
    // 1. Add an entry to the enum.
    // 2. Implement it in the switch statement below (Call::exec).
    // 3. Add a utility function to hsa/hsail-gpu-compute/util/magicinst.h,
    //    so its easy to call from an OpenCL kernel.

    // This enum should be identical to the enum in
    // hsa/hsail-gpu-compute/util/magicinst.h
    enum
    {
        MAGIC_PRINT_WF_32 = 0,
        MAGIC_PRINT_WF_64,
        MAGIC_PRINT_LANE,
        MAGIC_PRINT_LANE_64,
        MAGIC_PRINT_WF_FLOAT,
        MAGIC_SIM_BREAK,
        MAGIC_PREF_SUM,
        MAGIC_REDUCTION,
        MAGIC_MASKLANE_LOWER,
        MAGIC_MASKLANE_UPPER,
        MAGIC_JOIN_WF_BAR,
        MAGIC_WAIT_WF_BAR,
        MAGIC_PANIC,
        MAGIC_ATOMIC_NR_ADD_GLOBAL_U32_REG,
        MAGIC_ATOMIC_NR_ADD_GROUP_U32_REG,
        MAGIC_LOAD_GLOBAL_U32_REG,
        MAGIC_XACT_CAS_LD,
        MAGIC_MOST_SIG_THD,
        MAGIC_MOST_SIG_BROADCAST,
        MAGIC_PRINT_WFID_32,
        MAGIC_PRINT_WFID_64
    };

    void
    Call::execPseudoInst(Wavefront *w, GPUDynInstPtr gpuDynInst)
    {
        const VectorMask &mask = w->get_pred();

        int op = 0;
        bool got_op = false;

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int src_val0 = src1.get<int>(w, lane, 0);
                if (got_op) {
                    if (src_val0 != op) {
                        fatal("Multiple magic instructions per PC not "
                              "supported\n");
                    }
                } else {
                    op = src_val0;
                    got_op = true;
                }
            }
        }

        switch(op) {
          case MAGIC_PRINT_WF_32:
            MagicPrintWF32(w);
            break;
          case MAGIC_PRINT_WF_64:
            MagicPrintWF64(w);
            break;
          case MAGIC_PRINT_LANE:
            MagicPrintLane(w);
            break;
          case MAGIC_PRINT_LANE_64:
            MagicPrintLane64(w);
            break;
          case MAGIC_PRINT_WF_FLOAT:
            MagicPrintWFFloat(w);
            break;
          case MAGIC_SIM_BREAK:
            MagicSimBreak(w);
            break;
          case MAGIC_PREF_SUM:
            MagicPrefixSum(w);
            break;
          case MAGIC_REDUCTION:
            MagicReduction(w);
            break;
          case MAGIC_MASKLANE_LOWER:
            MagicMaskLower(w);
            break;
          case MAGIC_MASKLANE_UPPER:
            MagicMaskUpper(w);
            break;
          case MAGIC_JOIN_WF_BAR:
            MagicJoinWFBar(w);
            break;
          case MAGIC_WAIT_WF_BAR:
            MagicWaitWFBar(w);
            break;
          case MAGIC_PANIC:
            MagicPanic(w);
            break;

          // atomic instructions
          case MAGIC_ATOMIC_NR_ADD_GLOBAL_U32_REG:
            MagicAtomicNRAddGlobalU32Reg(w, gpuDynInst);
            break;

          case MAGIC_ATOMIC_NR_ADD_GROUP_U32_REG:
            MagicAtomicNRAddGroupU32Reg(w, gpuDynInst);
            break;

          case MAGIC_LOAD_GLOBAL_U32_REG:
            MagicLoadGlobalU32Reg(w, gpuDynInst);
            break;

          case MAGIC_XACT_CAS_LD:
            MagicXactCasLd(w);
            break;

          case MAGIC_MOST_SIG_THD:
            MagicMostSigThread(w);
            break;

          case MAGIC_MOST_SIG_BROADCAST:
            MagicMostSigBroadcast(w);
            break;

          case MAGIC_PRINT_WFID_32:
            MagicPrintWF32ID(w);
            break;

          case MAGIC_PRINT_WFID_64:
            MagicPrintWFID64(w);
            break;

          default: fatal("unrecognized magic instruction: %d\n", op);
        }
    }

    void
    Call::MagicPrintLane(Wavefront *w)
    {
    #if TRACING_ON
        const VectorMask &mask = w->get_pred();
        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);
                int src_val2 = src1.get<int>(w, lane, 2);
                if (src_val2) {
                    DPRINTFN("krl_prt (%s): CU%d, WF[%d][%d], lane %d: 0x%x\n",
                             disassemble(), w->computeUnit->cu_id, w->simdId,
                             w->wfSlotId, lane, src_val1);
                } else {
                    DPRINTFN("krl_prt (%s): CU%d, WF[%d][%d], lane %d: %d\n",
                             disassemble(), w->computeUnit->cu_id, w->simdId,
                             w->wfSlotId, lane, src_val1);
                }
            }
        }
    #endif
    }

    void
    Call::MagicPrintLane64(Wavefront *w)
    {
    #if TRACING_ON
        const VectorMask &mask = w->get_pred();
        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int64_t src_val1 = src1.get<int64_t>(w, lane, 1);
                int src_val2 = src1.get<int>(w, lane, 2);
                if (src_val2) {
                    DPRINTFN("krl_prt (%s): CU%d, WF[%d][%d], lane %d: 0x%x\n",
                             disassemble(), w->computeUnit->cu_id, w->simdId,
                             w->wfSlotId, lane, src_val1);
                } else {
                    DPRINTFN("krl_prt (%s): CU%d, WF[%d][%d], lane %d: %d\n",
                             disassemble(), w->computeUnit->cu_id, w->simdId,
                             w->wfSlotId, lane, src_val1);
                }
            }
        }
    #endif
    }

    void
    Call::MagicPrintWF32(Wavefront *w)
    {
    #if TRACING_ON
        const VectorMask &mask = w->get_pred();
        std::string res_str;
        res_str = csprintf("krl_prt (%s)\n", disassemble());

        for (int lane = 0; lane < VSZ; ++lane) {
            if (!(lane & 7)) {
                res_str += csprintf("DB%03d: ", (int)w->wfDynId);
            }

            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);
                int src_val2 = src1.get<int>(w, lane, 2);

                if (src_val2) {
                    res_str += csprintf("%08x", src_val1);
                } else {
                    res_str += csprintf("%08d", src_val1);
                }
            } else {
                res_str += csprintf("xxxxxxxx");
            }

            if ((lane & 7) == 7) {
                res_str += csprintf("\n");
            } else {
                res_str += csprintf(" ");
            }
        }

        res_str += "\n\n";
        DPRINTFN(res_str.c_str());
    #endif
    }

    void
    Call::MagicPrintWF32ID(Wavefront *w)
    {
    #if TRACING_ON
        const VectorMask &mask = w->get_pred();
        std::string res_str;
        int src_val3 = -1;
        res_str = csprintf("krl_prt (%s)\n", disassemble());

        for (int lane = 0; lane < VSZ; ++lane) {
            if (!(lane & 7)) {
                res_str += csprintf("DB%03d: ", (int)w->wfDynId);
            }

            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);
                int src_val2 = src1.get<int>(w, lane, 2);
                src_val3 = src1.get<int>(w, lane, 3);

                if (src_val2) {
                    res_str += csprintf("%08x", src_val1);
                } else {
                    res_str += csprintf("%08d", src_val1);
                }
            } else {
                res_str += csprintf("xxxxxxxx");
            }

            if ((lane & 7) == 7) {
                res_str += csprintf("\n");
            } else {
                res_str += csprintf(" ");
            }
        }

        res_str += "\n\n";
        if (w->wfDynId == src_val3) {
            DPRINTFN(res_str.c_str());
        }
    #endif
    }

    void
    Call::MagicPrintWF64(Wavefront *w)
    {
    #if TRACING_ON
        const VectorMask &mask = w->get_pred();
        std::string res_str;
        res_str = csprintf("krl_prt (%s)\n", disassemble());

        for (int lane = 0; lane < VSZ; ++lane) {
            if (!(lane & 3)) {
                res_str += csprintf("DB%03d: ", (int)w->wfDynId);
            }

            if (mask[lane]) {
                int64_t src_val1 = src1.get<int64_t>(w, lane, 1);
                int src_val2 = src1.get<int>(w, lane, 2);

                if (src_val2) {
                    res_str += csprintf("%016x", src_val1);
                } else {
                    res_str += csprintf("%016d", src_val1);
                }
            } else {
                res_str += csprintf("xxxxxxxxxxxxxxxx");
            }

            if ((lane & 3) == 3) {
                res_str += csprintf("\n");
            } else {
                res_str += csprintf(" ");
            }
        }

        res_str += "\n\n";
        DPRINTFN(res_str.c_str());
    #endif
    }

    void
    Call::MagicPrintWFID64(Wavefront *w)
    {
    #if TRACING_ON
        const VectorMask &mask = w->get_pred();
        std::string res_str;
        int src_val3 = -1;
        res_str = csprintf("krl_prt (%s)\n", disassemble());

        for (int lane = 0; lane < VSZ; ++lane) {
            if (!(lane & 3)) {
                res_str += csprintf("DB%03d: ", (int)w->wfDynId);
            }

            if (mask[lane]) {
                int64_t src_val1 = src1.get<int64_t>(w, lane, 1);
                int src_val2 = src1.get<int>(w, lane, 2);
                src_val3 = src1.get<int>(w, lane, 3);

                if (src_val2) {
                    res_str += csprintf("%016x", src_val1);
                } else {
                    res_str += csprintf("%016d", src_val1);
                }
            } else {
                res_str += csprintf("xxxxxxxxxxxxxxxx");
            }

            if ((lane & 3) == 3) {
                res_str += csprintf("\n");
            } else {
                res_str += csprintf(" ");
            }
        }

        res_str += "\n\n";
        if (w->wfDynId == src_val3) {
            DPRINTFN(res_str.c_str());
        }
    #endif
    }

    void
    Call::MagicPrintWFFloat(Wavefront *w)
    {
    #if TRACING_ON
        const VectorMask &mask = w->get_pred();
        std::string res_str;
        res_str = csprintf("krl_prt (%s)\n", disassemble());

        for (int lane = 0; lane < VSZ; ++lane) {
            if (!(lane & 7)) {
                res_str += csprintf("DB%03d: ", (int)w->wfDynId);
            }

            if (mask[lane]) {
                float src_val1 = src1.get<float>(w, lane, 1);
                res_str += csprintf("%08f", src_val1);
            } else {
                res_str += csprintf("xxxxxxxx");
            }

            if ((lane & 7) == 7) {
                res_str += csprintf("\n");
            } else {
                res_str += csprintf(" ");
            }
        }

        res_str += "\n\n";
        DPRINTFN(res_str.c_str());
    #endif
    }

    // raises a signal that GDB will catch
    // when done with the break, type "signal 0" in gdb to continue
    void
    Call::MagicSimBreak(Wavefront *w)
    {
        std::string res_str;
        // print out state for this wavefront and then break
        res_str = csprintf("Breakpoint encountered for wavefront %i\n",
                           w->wfSlotId);

        res_str += csprintf("  Kern ID: %i\n", w->kern_id);
        res_str += csprintf("  Phase ID: %i\n", w->simdId);
        res_str += csprintf("  Executing on CU #%i\n", w->computeUnit->cu_id);
        res_str += csprintf("  Exec mask: ");

        for (int i = VSZ - 1; i >= 0; --i) {
            if (w->execMask(i))
                res_str += "1";
            else
                res_str += "0";

            if ((i & 7) == 7)
                res_str += " ";
        }

        res_str += csprintf("(0x%016llx)\n", w->execMask().to_ullong());

        res_str += "\nHelpful debugging hints:\n";
        res_str += "   Check out w->s_reg / w->d_reg for register state\n";

        res_str += "\n\n";
        DPRINTFN(res_str.c_str());
        fflush(stdout);

        raise(SIGTRAP);
    }

    void
    Call::MagicPrefixSum(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        int res = 0;

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);
                dest.set<int>(w, lane, res);
                res += src_val1;
            }
        }
    }

    void
    Call::MagicReduction(Wavefront *w)
    {
        // reduction magic instruction
        //   The reduction instruction takes up to 64 inputs (one from
        //   each thread in a WF) and sums them. It returns the sum to
        //   each thread in the WF.
        const VectorMask &mask = w->get_pred();
        int res = 0;

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);
                res += src_val1;
            }
        }

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                dest.set<int>(w, lane, res);
            }
        }
    }

    void
    Call::MagicMaskLower(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        int res = 0;

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);

                if (src_val1) {
                    if (lane < (VSZ/2)) {
                        res = res | ((uint32_t)(1) << lane);
                    }
                }
            }
        }

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                dest.set<int>(w, lane, res);
            }
        }
    }

    void
    Call::MagicMaskUpper(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        int res = 0;
        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);

                if (src_val1) {
                    if (lane >= (VSZ/2)) {
                        res = res | ((uint32_t)(1) << (lane - (VSZ/2)));
                    }
                }
            }
        }

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                dest.set<int>(w, lane, res);
            }
        }
    }

    void
    Call::MagicJoinWFBar(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        int max_cnt = 0;

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                w->bar_cnt[lane]++;

                if (w->bar_cnt[lane] > max_cnt) {
                    max_cnt = w->bar_cnt[lane];
                }
            }
        }

        if (max_cnt > w->max_bar_cnt) {
            w->max_bar_cnt = max_cnt;
        }
    }

    void
    Call::MagicWaitWFBar(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        int max_cnt = 0;

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                w->bar_cnt[lane]--;
            }

            if (w->bar_cnt[lane] > max_cnt) {
                max_cnt = w->bar_cnt[lane];
            }
        }

        if (max_cnt < w->max_bar_cnt) {
            w->max_bar_cnt = max_cnt;
        }

        w->instructionBuffer.erase(w->instructionBuffer.begin() + 1,
                                   w->instructionBuffer.end());
        if (w->pendingFetch)
            w->dropFetch = true;
    }

    void
    Call::MagicPanic(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                int src_val1 = src1.get<int>(w, lane, 1);
                panic("OpenCL Code failed assertion #%d. Triggered by lane %s",
                      src_val1, lane);
            }
        }
    }

    void
    Call::calcAddr(Wavefront *w, GPUDynInstPtr m)
    {
        // the address is in src1 | src2
        for (int lane = 0; lane < VSZ; ++lane) {
            int src_val1 = src1.get<int>(w, lane, 1);
            int src_val2 = src1.get<int>(w, lane, 2);
            Addr addr = (((Addr) src_val1) << 32) | ((Addr) src_val2);

            m->addr[lane] = addr;
        }

    }

    void
    Call::MagicAtomicNRAddGlobalU32Reg(Wavefront *w, GPUDynInstPtr gpuDynInst)
    {
        GPUDynInstPtr m = gpuDynInst;

        calcAddr(w, m);

        for (int lane = 0; lane < VSZ; ++lane) {
            ((int*)m->a_data)[lane] = src1.get<int>(w, lane, 3);
        }

        m->m_op = brigAtomicToMemOpType(Brig::BRIG_OPCODE_ATOMICNORET,
                                        Brig::BRIG_ATOMIC_ADD);
        m->m_type = U32::memType;
        m->v_type = U32::vgprType;

        m->exec_mask = w->execMask();
        m->statusBitVector = 0;
        m->equiv = 0;  // atomics don't have an equivalence class operand
        m->n_reg = 1;
        m->memoryOrder = Enums::MEMORY_ORDER_NONE;
        m->scope = Enums::MEMORY_SCOPE_NONE;

        m->simdId = w->simdId;
        m->wfSlotId = w->wfSlotId;
        m->wfDynId = w->wfDynId;
        m->latency.init(&w->computeUnit->shader->tick_cnt);

        m->s_type = SEG_GLOBAL;
        m->pipeId = GLBMEM_PIPE;
        m->latency.set(w->computeUnit->shader->ticks(64));
        w->computeUnit->globalMemoryPipe.getGMReqFIFO().push(m);
        w->outstanding_reqs_wr_gm++;
        w->wr_gm_reqs_in_pipe--;
        w->outstanding_reqs_rd_gm++;
        w->rd_gm_reqs_in_pipe--;
        w->outstanding_reqs++;
        w->mem_reqs_in_pipe--;
    }

    void
    Call::MagicAtomicNRAddGroupU32Reg(Wavefront *w, GPUDynInstPtr gpuDynInst)
    {
        GPUDynInstPtr m = gpuDynInst;
        calcAddr(w, m);

        for (int lane = 0; lane < VSZ; ++lane) {
            ((int*)m->a_data)[lane] = src1.get<int>(w, lane, 1);
        }

        m->m_op = brigAtomicToMemOpType(Brig::BRIG_OPCODE_ATOMICNORET,
                                        Brig::BRIG_ATOMIC_ADD);
        m->m_type = U32::memType;
        m->v_type = U32::vgprType;

        m->exec_mask = w->execMask();
        m->statusBitVector = 0;
        m->equiv = 0;  // atomics don't have an equivalence class operand
        m->n_reg = 1;
        m->memoryOrder = Enums::MEMORY_ORDER_NONE;
        m->scope = Enums::MEMORY_SCOPE_NONE;

        m->simdId = w->simdId;
        m->wfSlotId = w->wfSlotId;
        m->wfDynId = w->wfDynId;
        m->latency.init(&w->computeUnit->shader->tick_cnt);

        m->s_type = SEG_GLOBAL;
        m->pipeId = GLBMEM_PIPE;
        m->latency.set(w->computeUnit->shader->ticks(64));
        w->computeUnit->globalMemoryPipe.getGMReqFIFO().push(m);
        w->outstanding_reqs_wr_gm++;
        w->wr_gm_reqs_in_pipe--;
        w->outstanding_reqs_rd_gm++;
        w->rd_gm_reqs_in_pipe--;
        w->outstanding_reqs++;
        w->mem_reqs_in_pipe--;
    }

    void
    Call::MagicLoadGlobalU32Reg(Wavefront *w, GPUDynInstPtr gpuDynInst)
    {
        GPUDynInstPtr m = gpuDynInst;
        // calculate the address
        calcAddr(w, m);

        m->m_op = Enums::MO_LD;
        m->m_type = U32::memType;  //MemDataType::memType;
        m->v_type = U32::vgprType; //DestDataType::vgprType;

        m->exec_mask = w->execMask();
        m->statusBitVector = 0;
        m->equiv = 0;
        m->n_reg = 1;
        m->memoryOrder = Enums::MEMORY_ORDER_NONE;
        m->scope = Enums::MEMORY_SCOPE_NONE;

        // FIXME
        //m->dst_reg = this->dest.regIndex();

        m->simdId = w->simdId;
        m->wfSlotId = w->wfSlotId;
        m->wfDynId = w->wfDynId;
        m->latency.init(&w->computeUnit->shader->tick_cnt);

        m->s_type = SEG_GLOBAL;
        m->pipeId = GLBMEM_PIPE;
        m->latency.set(w->computeUnit->shader->ticks(1));
        w->computeUnit->globalMemoryPipe.getGMReqFIFO().push(m);
        w->outstanding_reqs_rd_gm++;
        w->rd_gm_reqs_in_pipe--;
        w->outstanding_reqs++;
        w->mem_reqs_in_pipe--;
    }

    void
    Call::MagicXactCasLd(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        int src_val1 = 0;

        for (int lane = 0; lane < VSZ; ++lane) {
            if (mask[lane]) {
                src_val1 = src1.get<int>(w, lane, 1);
                break;
            }
        }

        if (!w->computeUnit->xactCasLoadMap.count(src_val1)) {
            w->computeUnit->xactCasLoadMap[src_val1] = ComputeUnit::waveQueue();
            w->computeUnit->xactCasLoadMap[src_val1].waveIDQueue.clear();
        }

        w->computeUnit->xactCasLoadMap[src_val1].waveIDQueue
            .push_back(ComputeUnit::waveIdentifier(w->simdId, w->wfSlotId));
    }

    void
    Call::MagicMostSigThread(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        unsigned mst = true;

        for (int lane = VSZ - 1; lane >= 0; --lane) {
            if (mask[lane]) {
                dest.set<int>(w, lane, mst);
                mst = false;
            }
        }
    }

    void
    Call::MagicMostSigBroadcast(Wavefront *w)
    {
        const VectorMask &mask = w->get_pred();
        int res = 0;
        bool got_res = false;

        for (int lane = VSZ - 1; lane >= 0; --lane) {
            if (mask[lane]) {
                if (!got_res) {
                    res = src1.get<int>(w, lane, 1);
                    got_res = true;
                }
                dest.set<int>(w, lane, res);
            }
        }
    }

} // namespace HsailISA
