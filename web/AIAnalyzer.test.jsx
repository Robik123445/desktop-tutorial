+import { render, screen, fireEvent } from '@testing-library/react';
+import AIAnalyzer from './src/AIAnalyzer';
+
+beforeEach(() => {
+  global.fetch = jest.fn(() => Promise.resolve({ json: () => Promise.resolve({ ok: true }) }));
+});
+
+test('posts optimize request on run', async () => {
+  render(<AIAnalyzer />);
+  fireEvent.click(screen.getByText('Run Analysis'));
+  expect(global.fetch).toHaveBeenCalledWith('/log', expect.any(Object));
+  expect(global.fetch).toHaveBeenCalledWith('/optimize', expect.any(Object));
+});
