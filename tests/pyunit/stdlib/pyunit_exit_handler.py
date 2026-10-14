# Copyright (c) 2026 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import unittest


class TestExitHandlerCompatibility(unittest.TestCase):
    def test_exit_hypercall_is_publicly_reexported(self):
        import _m5.event as _m5_event

        from gem5.simulate.exit_handler import ExitHypercall

        self.assertIs(ExitHypercall, _m5_event.ExitHypercall)
        self.assertEqual(
            int(ExitHypercall.SCHEDULED_EXIT.value),
            int(_m5_event.ExitHypercall.SCHEDULED_EXIT.value),
        )

    def test_resolve_hypercall_id(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        # Enum selector.
        enum_sel = exit_handler_mod.ExitHypercall.SCHEDULED_EXIT
        resolved_enum = exit_handler_mod._resolve_hypercall_id(enum_sel)
        self.assertEqual(resolved_enum, int(enum_sel.value))

        member = exit_handler_mod.ExitHypercall.CLASSIC_GENERATOR

        # Integer selector, including custom/user-defined IDs.
        resolved_int = exit_handler_mod._resolve_hypercall_id(
            int(member.value)
        )
        self.assertEqual(resolved_int, int(member.value))
        self.assertEqual(exit_handler_mod._resolve_hypercall_id(4096), 4096)
        self.assertEqual(exit_handler_mod._resolve_hypercall_id(0), 0)
        self.assertEqual(
            exit_handler_mod._resolve_hypercall_id((1 << 64) - 1),
            (1 << 64) - 1,
        )

    def test_resolve_hypercall_id_rejects_unknown_selectors(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        with self.assertRaisesRegex(TypeError, "ExitHypercall enum"):
            exit_handler_mod._resolve_hypercall_id("not-a-hypercall")

        with self.assertRaisesRegex(TypeError, "ExitHypercall enum"):
            exit_handler_mod._resolve_hypercall_id(object())
        with self.assertRaisesRegex(TypeError, "not bool"):
            exit_handler_mod._resolve_hypercall_id(True)
        with self.assertRaisesRegex(ValueError, "unsigned 64-bit"):
            exit_handler_mod._resolve_hypercall_id(-1)
        with self.assertRaisesRegex(ValueError, "unsigned 64-bit"):
            exit_handler_mod._resolve_hypercall_id(1 << 64)

    def test_exit_handler_class_registration_accepts_exit_hypercall(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        h_id = int(exit_handler_mod.ExitHypercall.KERNEL_BOOTED.value)
        handler_map = exit_handler_mod.ExitHandler.get_handler_map()
        existing = handler_map.get(h_id)

        try:

            class EnumRegisteredHandler(
                exit_handler_mod.ExitHandler,
                hypercall=exit_handler_mod.ExitHypercall.KERNEL_BOOTED,
            ):
                def _process(self, simulator):
                    pass

                def _exit_simulation(self):
                    return False

            self.assertIs(handler_map[h_id], EnumRegisteredHandler)
            self.assertEqual(EnumRegisteredHandler.get_handler_id(), h_id)

        finally:
            if existing is None:
                handler_map.pop(h_id, None)
            else:
                handler_map[h_id] = existing

    def test_exit_handler_class_registration_accepts_legacy_hypercall_num(
        self,
    ):
        from gem5.simulate import exit_handler as exit_handler_mod

        h_id = int(exit_handler_mod.ExitHypercall.KERNEL_BOOTED.value)
        handler_map = exit_handler_mod.ExitHandler.get_handler_map()
        existing = handler_map.get(h_id)

        try:

            class LegacyRegisteredHandler(
                exit_handler_mod.ExitHandler,
                hypercall_num=h_id,
            ):
                def _process(self, simulator):
                    pass

                def _exit_simulation(self):
                    return False

            self.assertIs(handler_map[h_id], LegacyRegisteredHandler)
            self.assertEqual(LegacyRegisteredHandler.get_handler_id(), h_id)

        finally:
            if existing is None:
                handler_map.pop(h_id, None)
            else:
                handler_map[h_id] = existing

    def test_exit_handler_subclass_inherits_registered_hypercall(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        h_id = int(exit_handler_mod.ExitHypercall.KERNEL_BOOTED.value)
        handler_map = exit_handler_mod.ExitHandler.get_handler_map()
        existing = handler_map.get(h_id)

        try:

            class CustomKernelBootedExitHandler(
                exit_handler_mod.KernelBootedExitHandler
            ):
                def _process(self, simulator):
                    pass

            self.assertIs(handler_map[h_id], CustomKernelBootedExitHandler)
            self.assertEqual(
                CustomKernelBootedExitHandler.get_handler_id(), h_id
            )

        finally:
            if existing is None:
                handler_map.pop(h_id, None)
            else:
                handler_map[h_id] = existing

    def test_exit_handler_class_registration_rejects_ambiguous_selector(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        with self.assertRaisesRegex(TypeError, "Specify only one"):

            class AmbiguousExitHandler(
                exit_handler_mod.ExitHandler,
                hypercall=exit_handler_mod.ExitHypercall.WORK_BEGIN,
                hypercall_id=int(
                    exit_handler_mod.ExitHypercall.WORK_END.value
                ),
            ):
                def _process(self, simulator):
                    pass

                def _exit_simulation(self):
                    return False

    def test_register_exit_handler_and_invocation(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        h_id = int(exit_handler_mod.ExitHypercall.CLASSIC_GENERATOR.value)

        invocations = []

        def handler_func(simulator, payload):
            invocations.append((simulator, payload))
            return True

        handler_map = exit_handler_mod.ExitHandler.get_handler_map()
        existing = handler_map.get(h_id)

        try:
            handler_cls = exit_handler_mod.register_exit_handler(
                exit_handler_mod.ExitHypercall.CLASSIC_GENERATOR,
                handler_func,
                "test handler",
            )

            payload = {"cause": "test", "code": "0"}
            handler = handler_cls(payload)
            result = handler.handle(None)
            self.assertTrue(result)
            self.assertEqual(handler.get_handler_description(), "test handler")
            self.assertEqual(len(invocations), 1)
            self.assertEqual(invocations[0][1], payload)

        finally:
            if existing is None:
                handler_map.pop(h_id, None)
            else:
                handler_map[h_id] = existing

    def test_register_exit_handler_supports_hypercall_id_alias(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        h_id = int(exit_handler_mod.ExitHypercall.WORK_END.value)
        handler_map = exit_handler_mod.ExitHandler.get_handler_map()
        existing = handler_map.get(h_id)

        try:
            handler_cls = exit_handler_mod.register_exit_handler(
                None,
                lambda simulator, payload: False,
                "custom hypercall_id handler",
                hypercall_id=int(
                    exit_handler_mod.ExitHypercall.WORK_END.value
                ),
            )

            self.assertIs(handler_map[h_id], handler_cls)
            self.assertFalse(handler_cls({}).handle(None))

        finally:
            if existing is None:
                handler_map.pop(h_id, None)
            else:
                handler_map[h_id] = existing

    def test_register_exit_handler_rejects_missing_or_ambiguous_selector(self):
        from gem5.simulate import exit_handler as exit_handler_mod

        def handler_func(simulator, payload):
            return False

        with self.assertRaisesRegex(ValueError, "must be provided"):
            exit_handler_mod.register_exit_handler(
                None, handler_func, "missing hypercall"
            )

        with self.assertRaisesRegex(ValueError, "Specify only one"):
            exit_handler_mod.register_exit_handler(
                exit_handler_mod.ExitHypercall.WORK_BEGIN,
                handler_func,
                "ambiguous hypercall",
                hypercall_id=int(
                    exit_handler_mod.ExitHypercall.WORK_END.value
                ),
            )

    def test_classic_generator_uses_payload_cause_when_present(self):
        from gem5.simulate.exit_event import ExitEvent
        from gem5.simulate.exit_handler import ClassicGeneratorExitHandler

        self._with_classic_generator_maps(
            ExitEvent.MAX_TICK,
            payload={"cause": "simulate() limit reached", "code": "0"},
            legacy_cause="legacy cause should not be read",
            should_read_legacy_cause=False,
        )

    def test_classic_generator_falls_back_to_legacy_cause(self):
        from gem5.simulate.exit_event import ExitEvent
        from gem5.simulate.exit_handler import ClassicGeneratorExitHandler

        simulator = self._with_classic_generator_maps(
            ExitEvent.SCHEDULED_TICK,
            payload={"code": "0"},
            legacy_cause="Tick exit reached",
            should_read_legacy_cause=True,
        )
        self.assertEqual(simulator.legacy_cause_reads, 1)

    def _with_classic_generator_maps(
        self,
        exit_event,
        payload,
        legacy_cause,
        should_read_legacy_cause,
    ):
        from gem5.simulate import exit_handler as exit_handler_mod

        handler_cls = exit_handler_mod.ClassicGeneratorExitHandler
        missing = object()
        old_on_exit_event = getattr(handler_cls, "_on_exit_event", missing)
        old_default_on_exit_dict = getattr(
            handler_cls, "_default_on_exit_dict", missing
        )
        old_expected_execution_order = getattr(
            handler_cls, "_expected_execution_order", missing
        )

        def restore_attr(name, value):
            if value is missing:
                try:
                    delattr(handler_cls, name)
                except AttributeError:
                    pass
            else:
                setattr(handler_cls, name, value)

        def single_yield(value):
            yield value

        class FakeSimulator:
            def __init__(self):
                self._exit_event_count = 0
                self._tick_stopwatch = []
                self.legacy_cause_reads = 0

            def get_current_tick(self):
                return 100

            def get_last_exit_event_cause(self):
                self.legacy_cause_reads += 1
                if not should_read_legacy_cause:
                    raise AssertionError(
                        "payload cause should avoid legacy cause lookup"
                    )
                return legacy_cause

        try:
            handler_cls._on_exit_event = {exit_event: single_yield(False)}
            handler_cls._default_on_exit_dict = {
                exit_event: single_yield(True)
            }
            handler_cls._expected_execution_order = [exit_event]

            simulator = FakeSimulator()
            handler = handler_cls(payload)
            self.assertFalse(handler.handle(simulator))
            self.assertEqual(simulator._tick_stopwatch, [(exit_event, 100)])
            return simulator

        finally:
            restore_attr("_on_exit_event", old_on_exit_event)
            restore_attr("_default_on_exit_dict", old_default_on_exit_dict)
            restore_attr(
                "_expected_execution_order", old_expected_execution_order
            )

    def test_exitSimLoop_forwards_to_exitSimulationLoopClassic(self):
        # Ensure the legacy helper preserves legacy scheduling arguments while
        # forwarding through the classic hypercall-aware path.
        import m5.event as m5_event

        import _m5.event as _m5_event

        original = _m5_event.exitSimulationLoopClassic
        calls = []

        def fake_exitSimulationLoopClassic(
            message,
            exit_code,
            when,
            repeat,
            serialize,
        ):
            calls.append(
                (
                    message,
                    exit_code,
                    when,
                    repeat,
                    serialize,
                )
            )

        try:
            _m5_event.exitSimulationLoopClassic = (
                fake_exitSimulationLoopClassic
            )

            with self.assertWarnsRegex(
                DeprecationWarning, "exitSimulationLoopClassic"
            ):
                m5_event.exitSimLoop(
                    "legacy-cause",
                    exit_code=7,
                    when=123,
                    repeat=11,
                    serialize=True,
                )

            self.assertEqual(len(calls), 1)
            (
                message,
                exit_code,
                tick,
                repeat,
                serialize,
            ) = calls[0]
            self.assertEqual(message, "legacy-cause")
            self.assertEqual(exit_code, 7)
            self.assertEqual(tick, 123)
            self.assertEqual(repeat, 11)
            self.assertTrue(serialize)

        finally:
            _m5_event.exitSimulationLoopClassic = original

    def test_exitSimLoop_defaults_when_to_native_curTick(self):
        # With no explicit tick, leave the default as None so the native
        # helper resolves curTick() inside the embedded runtime.
        import m5.event as m5_event

        import _m5.event as _m5_event

        original = _m5_event.exitSimulationLoopClassic
        calls = []

        def fake_exitSimulationLoopClassic(
            message,
            exit_code,
            when,
            repeat,
            serialize,
        ):
            calls.append((when, repeat))

        try:
            _m5_event.exitSimulationLoopClassic = (
                fake_exitSimulationLoopClassic
            )

            with self.assertWarnsRegex(
                DeprecationWarning, "exitSimulationLoopClassic"
            ):
                m5_event.exitSimLoop("legacy-cause")

            self.assertEqual(calls, [(None, 0)])

        finally:
            _m5_event.exitSimulationLoopClassic = original

    def test_public_exit_helpers_forward_to_native_bindings(self):
        import m5.event as m5_event

        import _m5.event as _m5_event

        original_classic = _m5_event.exitSimulationLoopClassic
        original_hypercall = _m5_event.exitSimulationLoop
        calls = []

        try:
            _m5_event.exitSimulationLoopClassic = lambda *args: calls.append(
                ("classic", args)
            )
            _m5_event.exitSimulationLoop = lambda *args: calls.append(
                ("hypercall", args)
            )

            m5_event.exitSimulationLoopClassic("cause", 3, 10, 4, False)
            m5_event.exitSimulationLoop(4096, {"key": "value"}, 20, 5)

            self.assertEqual(
                calls,
                [
                    ("classic", ("cause", 3, 10, 4, False)),
                    ("hypercall", (4096, {"key": "value"}, 20, 5)),
                ],
            )
            self.assertIn("exitSimulationLoop", m5_event.__all__)
            self.assertIn("exitSimulationLoopClassic", m5_event.__all__)

        finally:
            _m5_event.exitSimulationLoopClassic = original_classic
            _m5_event.exitSimulationLoop = original_hypercall

    def test_default_tick_exit_uses_compatibility_path(self):
        # The internal scheduled-tick helper should no longer call the
        # deprecated pybind exitSimLoop entry point, but it must still preserve
        # the classic ExitEvent.SCHEDULED_TICK cause for existing users.
        import importlib

        import _m5.event as _m5_event

        m5_simulate = importlib.import_module("m5.simulate")

        original_exitSimulationLoopClassic = (
            _m5_event.exitSimulationLoopClassic
        )
        original_curTick = m5_simulate.curTick
        calls = []

        def fake_exitSimLoop(*args, **kwargs):
            raise AssertionError("deprecated exitSimLoop should not be called")

        def fake_exitSimulationLoopClassic(
            message,
            exit_code,
            when,
            repeat,
            serialize,
        ):
            calls.append(
                (
                    message,
                    exit_code,
                    when,
                    repeat,
                    serialize,
                )
            )

        try:
            original_exitSimLoop = getattr(_m5_event, "exitSimLoop", None)
            if original_exitSimLoop is not None:
                _m5_event.exitSimLoop = fake_exitSimLoop
            _m5_event.exitSimulationLoopClassic = (
                fake_exitSimulationLoopClassic
            )
            m5_simulate.curTick = lambda: 0

            m5_simulate.scheduleTickExitAbsolute(123)

            self.assertEqual(len(calls), 1)
            (
                message,
                exit_code,
                when,
                repeat,
                serialize,
            ) = calls[0]
            self.assertEqual(message, "Tick exit reached")
            self.assertEqual(exit_code, 0)
            self.assertEqual(when, 123)
            self.assertEqual(repeat, 0)
            self.assertFalse(serialize)

        finally:
            if original_exitSimLoop is not None:
                _m5_event.exitSimLoop = original_exitSimLoop
            _m5_event.exitSimulationLoopClassic = (
                original_exitSimulationLoopClassic
            )
            m5_simulate.curTick = original_curTick
