import { render, screen, fireEvent } from '@testing-library/react';
 import GCodeExporter from './src/GCodeExporter';
 
 beforeEach(() => {
-  global.fetch = jest.fn(() => Promise.resolve({
-    json: () => Promise.resolve({ gcode: 'G1 X0 Y0', ok: true })
-  }));
+  global.fetch = jest.fn((url) => {
+    if (url === '/export') {
+      return Promise.resolve({ json: () => Promise.resolve({ gcode: 'G1 X0 Y0' }) });
+    }
+    if (url === '/send') {
+      return Promise.resolve({ json: () => Promise.resolve({ ok: true }) });
+    }
+    // /ports or /log
+    return Promise.resolve({ json: () => Promise.resolve(['/dev/ttyUSB0']) });
+  });
 });
 
 test('exports and sends gcode', async () => {
   render(<GCodeExporter />);
   fireEvent.change(screen.getByPlaceholderText(/Toolpath JSON/i), {
     target: { value: '[{"x":0,"y":0}]' }
   });
   fireEvent.change(screen.getByPlaceholderText(/File name/i), {
     target: { value: 'demo' }
   });
   fireEvent.click(screen.getByText('Generate G-code'));
+  expect(global.fetch).toHaveBeenCalledWith('/ports');
   expect(global.fetch).toHaveBeenCalledWith('/log', expect.any(Object));
   expect(global.fetch).toHaveBeenCalledWith('/export', expect.any(Object));
   await screen.findByText(/G1 X0 Y0/);
   fireEvent.click(screen.getByText('Send to Machine'));
   expect(global.fetch).toHaveBeenCalledWith('/send', expect.any(Object));
 });
