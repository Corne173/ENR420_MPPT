import csv
import tempfile
import unittest
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import patch

from mppt_serial_gui import (
    CSV_HEADER,
    MpptSerialGui,
    TelemetryCsvRecorder,
    TelemetryPacket,
)


class FakeWidget:
    def __init__(self) -> None:
        self.options = {}

    def configure(self, **options) -> None:
        self.options.update(options)


class FakeVariable:
    def __init__(self) -> None:
        self.value = None

    def set(self, value) -> None:
        self.value = value


class FakeReader:
    def __init__(self) -> None:
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True


class TelemetryCsvRecorderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.csv_path = Path(self.temporary_directory.name) / 'telemetry.csv'
        self.recorder = TelemetryCsvRecorder()

    def tearDown(self) -> None:
        self.recorder.stop()
        self.temporary_directory.cleanup()

    def read_rows(self) -> list[list[str]]:
        with self.csv_path.open(encoding='utf-8', newline='') as csv_file:
            return list(csv.reader(csv_file))

    def test_header_and_packet_are_flushed_before_stop(self) -> None:
        self.recorder.start(self.csv_path)
        self.assertEqual(self.read_rows(), [list(CSV_HEADER)])

        self.recorder.write(
            TelemetryPacket(
                unix_ms=1_700_000_000_123,
                i_in_raw=804,
                i_out_raw=2030,
                v_out_raw=1200,
                v_in_raw=1100,
                valid=True,
                temp0_c=25.5,
                temp1_c=None,
                irr_w_m2=850.25,
                state='RUN',
                fault='NONE',
            )
        )

        rows = self.read_rows()
        self.assertEqual(rows[0], list(CSV_HEADER))
        self.assertEqual(
            rows[1],
            [
                '1700000000123',
                '804',
                '2030',
                '1200',
                '1100',
                '1',
                '25.5',
                '',
                '850.25',
                'RUN',
                'NONE',
                '',
                '',
                '',
                '',
                '',
                '',
                '',
            ],
        )
        self.assertEqual(self.recorder.row_count, 1)

    def test_legacy_packet_writes_blank_sensor_fields(self) -> None:
        packet = MpptSerialGui._parse_telemetry_packet(
            None,
            'ADC,123,7,804,2030,1200,1100,1,RUN,NONE',
            1_700_000_000_456,
        )
        self.assertIsNotNone(packet)

        self.recorder.start(self.csv_path)
        self.recorder.write(packet)

        row = self.read_rows()[1]
        self.assertEqual(row[0:6], ['1700000000456', '804', '2030', '1200', '1100', '1'])
        self.assertEqual(row[6:9], ['', '', ''])
        self.assertEqual(row[9:11], ['RUN', 'NONE'])

    def test_extended_packet_parses_and_records_mppt_status(self) -> None:
        packet = MpptSerialGui._parse_telemetry_packet(
            None,
            (
                '804,2030,1200,1100,1,25.50,NA,850,RUN,NONE,'
                'SAMPLE,0.1125,-0.0025,0.1125,0.0000,101.25,100.75'
            ),
            1_700_000_000_789,
        )

        self.assertIsNotNone(packet)
        self.assertEqual(packet.mppt_phase, 'SAMPLE')
        self.assertAlmostEqual(packet.mppt_gain, 0.1125)
        self.assertAlmostEqual(packet.mppt_step, -0.0025)
        self.assertAlmostEqual(packet.reference_power_w, 101.25)
        self.assertAlmostEqual(packet.sampled_power_w, 100.75)

        self.recorder.start(self.csv_path)
        self.recorder.write(packet)
        row = self.read_rows()[1]
        self.assertEqual(
            row[11:],
            ['SAMPLE', '0.1125', '-0.0025', '0.1125', '0.0', '101.25', '100.75'],
        )

    def test_disconnect_stops_and_closes_recording(self) -> None:
        self.recorder.start(self.csv_path)
        reader = FakeReader()
        record_button = FakeWidget()
        connect_button = FakeWidget()
        connection_status = FakeVariable()
        messages = []
        gui = SimpleNamespace(
            recorder=self.recorder,
            reader=reader,
            record_button=record_button,
            connect_button=connect_button,
            connection_status=connection_status,
            _append_terminal=messages.append,
        )
        gui.stop_recording = MethodType(MpptSerialGui.stop_recording, gui)

        MpptSerialGui.disconnect(gui)

        self.assertFalse(self.recorder.is_recording)
        self.assertTrue(reader.stopped)
        self.assertIsNone(gui.reader)
        self.assertEqual(record_button.options['state'], 'disabled')
        self.assertEqual(connect_button.options['text'], 'Connect')
        self.assertEqual(connection_status.value, 'Disconnected')
        self.assertIn('0 telemetry rows saved', messages[0])

    def test_cancelling_save_dialog_does_not_start_recording(self) -> None:
        gui = SimpleNamespace(
            recorder=self.recorder,
            reader=SimpleNamespace(serial_port=SimpleNamespace(is_open=True)),
        )

        with patch('mppt_serial_gui.filedialog.asksaveasfilename', return_value=''):
            MpptSerialGui.start_recording(gui)

        self.assertFalse(self.recorder.is_recording)
        self.assertFalse(self.csv_path.exists())


if __name__ == '__main__':
    unittest.main()
