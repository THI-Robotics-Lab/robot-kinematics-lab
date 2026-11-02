# Offline operation

The application runs entirely in the browser and makes no runtime API, CDN,
font or robot-asset requests. It needs WebGL and a local HTTP server; opening
`dist/index.html` as a `file://` URL is not supported by browser module rules.

While online, install Node.js and npm and build the application:

```bash
npm ci
npm run build
```

Copy the **entire** `dist/` directory to the offline computer. It contains the
bundled JavaScript, CSS and third-party license notices. Serve that directory
over HTTP using any local static file server. If Node.js and the project's
installed development dependencies are available on the offline machine, from
the project root run:

```bash
npm run preview
```

Open the local address printed by Vite and stop the server with Ctrl+C. The
server only delivers static files; the app has no backend. If copying only
`dist/`, use an available static file server on that machine instead. A fresh
installation or browser download needs Internet access unless the required npm
and Playwright caches were prepared in advance. There is no service worker or
installed offline cache; a remote-host reload still needs that host.
