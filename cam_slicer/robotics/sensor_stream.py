def start(self) -> None:
        """Start background reading thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("SensorStream started mode=%s source=%s", self.mode, self.source)

    def stop(self) -> None:
        """Stop reading thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        logger.info("SensorStream stopped")

    # Internal helpers -----------------------------------------------------
    def _run(self) -> None:
        if self.mode == "websocket":
            asyncio.run(self._run_ws())
        else:
            self._run_serial()

    def _run_serial(self) -> None:
        try:
            import serial
        except ModuleNotFoundError as exc:  # pragma: no cover - optional
            logger.error("pyserial not installed: %s", exc)
            return

        try:
            with serial.Serial(self.source, self.baud, timeout=1) as ser:
                while self._running:
                    line = ser.readline().decode().strip()
                    if line:
                        self._handle_line(line)
        except Exception as exc:  # pragma: no cover - serial port errors
            logger.error("Serial error: %s", exc)

    async def _run_ws(self) -> None:
        import websockets

        try:
            async with websockets.connect(self.source) as ws:
                while self._running:
                    line = await ws.recv()
                    if line:
                        self._handle_line(line)
        except Exception as exc:  # pragma: no cover - connection errors
            logger.error("WebSocket error: %s", exc)

    def _handle_line(self, line: str) -> None:
        try:
            parts = [float(x) for x in line.replace(",", " ").split()]
