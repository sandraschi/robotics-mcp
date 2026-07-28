import { defineConfig } from "@playwright/test";
export default defineConfig({
  testDir: "./e2e",
  timeout: 60000,
  retries: 1,
  use: {
    baseURL: "http://localhost:10707",
    headless: true,
    screenshot: "only-on-failure",
  },
  webServer: {
    command: "uv run python -m robotics_mcp.server --port 10706",
    port: 10706,
    timeout: 30000,
    reuseExistingServer: false,
  },
});
