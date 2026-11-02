# Development and contribution

The application runs in the browser. No Python, ROS or Docker is required.
See the [technical guide](docs/TECHNICAL.md) and [validation](docs/VALIDATION.md)
for model and test details.
Clone the public repository and use Node.js 22.16.0 and npm:

```bash
git clone https://github.com/THI-Robotics-Lab/robot-kinematics-lab.git
cd robot-kinematics-lab
npm ci
npm run dev
```

Open the URL Vite prints, normally <http://127.0.0.1:5173>. Press Ctrl+C to
stop the server. Initial installation needs Internet access or an npm cache.

Install Node.js and npm using a Node installer or a version manager. On Linux,
macOS or WSL, [nvm](https://github.com/nvm-sh/nvm) can use the version pinned
in `.nvmrc` via `nvm install` and `nvm use`. On Windows use a Node installer,
a Windows-compatible version manager, or nvm within WSL. nvm manages Node.js
versions; npm installs this project's dependencies into local `node_modules/`.

## Checks

```bash
npm test                         # numerical checks
npm run build                    # TypeScript and production build
npx playwright install chromium # once per browser version
npm run test:browser             # tests the production build on localhost
```

Minimal Linux/WSL systems may need OS libraries; Playwright provides
`npx playwright install-deps chromium` (system installation may require admin
permission). The browser tests use local port 4173. See [OFFLINE.md](OFFLINE.md)
for serving a production build without Internet access.

## Propose a change

Fork the public repository, create a feature branch, make a focused change and
run the checks above. Open a pull request against public `main` describing the
change and its validation. Report issues through the public repository. The
maintainers review contributions before inclusion. The Pages workflow runs
checks and deploys the production bundle on pushes to `main`.
