import path from "node:path";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
  server: {
    host: "127.0.0.1",
    port: 10706,
    strictPort: true,
    proxy: {
      "/api": {
        target: "http://127.0.0.1:10707",
        changeOrigin: true,
        secure: false,
      },
    },
  },
});
