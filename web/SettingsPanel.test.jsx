+import { render, screen, fireEvent } from '@testing-library/react';
+import SettingsPanel from './src/SettingsPanel';
+
+beforeEach(() => {
+  global.fetch = jest.fn(() => Promise.resolve());
+  localStorage.clear();
+});
+
+test('saves settings to localStorage and logs', () => {
+  render(<SettingsPanel />);
+  fireEvent.change(screen.getByText('Light'), { target: { value: 'Dark' } });
+  fireEvent.change(screen.getByPlaceholderText('http://localhost:8000'), {
+    target: { value: 'http://api' }
+  });
+  fireEvent.click(screen.getByText('Save'));
+  expect(localStorage.getItem('theme')).toBe('Dark');
+  expect(localStorage.getItem('apiUrl')).toBe('http://api');
+  expect(global.fetch).toHaveBeenCalledWith('/log', expect.any(Object));
+  expect(screen.getByText('Settings saved')).toBeInTheDocument();
+});
