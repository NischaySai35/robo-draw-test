import { defineConfig, loadEnv } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig(({ mode }) => {
  // loadEnv, not process.env: Vite only exposes VITE_-prefixed vars to the
  // client, and this key must NOT be one of those. It is read here, in the
  // Node-side config, and injected by the proxy -- so it never reaches the
  // browser bundle at all.
  const env = loadEnv(mode, process.cwd(), '')
  const apiKey = env.ANTHROPIC_API_KEY ?? ''

  return {
    plugins: [react()],
    server: {
      proxy: {
        // The "Prompt to Build" tab posts here instead of straight to
        // api.anthropic.com, which keeps the key server-side and sidesteps CORS.
        // With no key configured the request still goes through and the API's
        // own 401 comes back, which says more than a guess made here would.
        '/api/anthropic': {
          target: 'https://api.anthropic.com',
          changeOrigin: true,
          rewrite: (path) => path.replace(/^\/api\/anthropic/, ''),
          configure: (proxy) => {
            proxy.on('proxyReq', (proxyReq) => {
              if (apiKey) proxyReq.setHeader('x-api-key', apiKey)
              proxyReq.setHeader('anthropic-version', '2023-06-01')
              // Anthropic rejects an Origin it doesn't expect; this is a
              // server-to-server call now, so drop the browser's.
              proxyReq.removeHeader('origin')
              proxyReq.removeHeader('referer')
            })
          },
        },
        // Ollama needs no key at all -- this exists purely to dodge the
        // browser's CORS preflight against a bare localhost port.
        '/api/ollama': {
          target: 'http://127.0.0.1:11434',
          changeOrigin: true,
          rewrite: (path) => path.replace(/^\/api\/ollama/, ''),
        },
      },
    },
  }
})
