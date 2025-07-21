def resume(self) -> None:
        """Resume sending commands."""
        self._paused = False
        logging.info("Streaming resumed")

    def stop(self) -> None:
        """Stop streaming."""
        self._stop = True
        logging.info("Streaming stopped by user")

    def stream(self) -> None:
        """Send the configured G-code file."""
        if serial is None:
            raise ImportError("pyserial is required for streaming")

        with serial.Serial(self.port, self.baud, timeout=1) as ser:
            lines = self.path.read_text().splitlines()
            idx = 0
            while idx < len(lines):
                if self._stop:
                    break
                if self._paused:
                    time.sleep(0.1)
                    continue
                line = lines[idx].strip()
                if not line:
                    idx += 1
                    continue
                ser.write((line + "\n").encode())
                _wait_for_ok(ser)
                idx += 1
