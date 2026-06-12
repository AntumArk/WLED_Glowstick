Import("env")
import shutil
import os

node_ex = shutil.which("node")
docker_ex = shutil.which("docker")

# Skip rebuild if all generated headers already exist
_headers = [
    "wled00/html_ui.h", "wled00/html_settings.h", "wled00/html_other.h",
    "wled00/js_iro.h",
]
_project_dir = env.subst("$PROJECT_DIR")
_already_built = all(os.path.isfile(os.path.join(_project_dir, h)) for h in _headers)

if _already_built:
    print('\x1b[6;33;42m' + 'Web UI is already built' + '\x1b[0m')

elif node_ex is not None:
    # Node.js is available locally — use it directly.
    print('\x1b[6;33;42m' + 'Installing node packages' + '\x1b[0m')
    env.Execute("npm ci")
    exitCode = env.Execute("npm run build")
    if exitCode:
        print('\x1b[0;31;43m' + 'npm run build failed — check https://kno.wled.ge/advanced/compiling-wled/' + '\x1b[0m')
        exit(exitCode)

elif docker_ex is not None:
    # No local Node.js, but Docker is available — build inside a container.
    print('\x1b[6;33;42m' + 'Node.js not found, falling back to Docker for web UI build' + '\x1b[0m')
    exitCode = env.Execute("docker compose run --rm build-ui")
    if exitCode:
        print('\x1b[0;31;43m' + 'Docker web UI build failed. Ensure Docker Desktop is running and docker-compose.yml is present.' + '\x1b[0m')
        exit(exitCode)

else:
    print('\x1b[0;31;43m' + 'Neither Node.js nor Docker found. Cannot build web UI. See https://kno.wled.ge/advanced/compiling-wled/' + '\x1b[0m')
    exit(1)
