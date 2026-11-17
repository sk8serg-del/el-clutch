# Выкладывает текущие страницы из web/ на https://tools.bulbaracing.com/dev/ для проверки.
# В коммит попадает только папка web/dev: остальные правки, основной сайт и прошивка
# не затрагиваются, сборка прошивки не запускается.
#
#   python deploy_dev.py          — собрать и отправить
#   python deploy_dev.py --dry    — только собрать в web/dev, ничего не отправлять
#   python deploy_dev.py --page throttle.html — выложить только эту страницу
import datetime
import os
import shutil
import subprocess
import sys

root_dir = os.path.dirname(os.path.abspath(__file__))
web_dir = os.path.join(root_dir, "web")
dev_dir = os.path.join(web_dir, "dev")

# страница в web/dev -> исходник
PAGES = {p: os.path.join(web_dir, p) for p in ["index.html", "Clutch and Throttle Settings.html", "flash.html"]}
# BLE-страница ручки газа (проект throttle рядом с CLUTCH)
PAGES["throttle.html"] = os.path.join(root_dir, "..", "throttle", "tools", "web", "ble.html")

dry = "--dry" in sys.argv
if "--page" in sys.argv:
    only = sys.argv[sys.argv.index("--page") + 1]
    if only not in PAGES:
        sys.exit("Unknown page: " + only + " (known: " + ", ".join(PAGES) + ")")
    PAGES = {only: PAGES[only]}


def git(*args):
    return subprocess.run(["git", *args], cwd=root_dir, check=True, text=True,
                          encoding="utf-8", capture_output=True).stdout.strip()


if not dry:
    # Отправка не должна увезти чужое и не должна затирать новое на GitHub
    if git("diff", "--cached", "--name-only"):
        sys.exit("Stopped: something is already staged, commit would be mixed")
    git("fetch", "origin", "main")
    if git("rev-list", "--count", "HEAD..origin/main") != "0":
        sys.exit("Stopped: GitHub has newer commits")
    if git("rev-list", "--count", "origin/main..HEAD") != "0":
        sys.exit("Stopped: local commits not yet pushed would go along")

# Время выкладки внизу страницы: видно, что загрузилась новая версия
stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
footer = ('<p style="text-align:center; font-size:12px; color:#888; margin:16px 0;">'
          "dev build " + stamp + "</p>\n</body>")

# Ссылки только в dev-копии главной (основной сайт не меняется)
DEV_LINKS = (
    '    <a class="tool" href="throttle.html" style="border-left-color:#ff9800">\n'
    '      <b>Throttle (dev)</b>\n'
    '      <span>Contactless throttle: live sensor data, output voltages, calibration. '
    'Bluetooth, phone or computer.</span>\n'
    '    </a>\n'
)

os.makedirs(dev_dir, exist_ok=True)
for page, src in PAGES.items():
    with open(src, encoding="utf-8", newline="") as f:
        html = f.read()
    if page == "index.html":
        at = html.rindex("  </div>")
        html = html[:at] + DEV_LINKS + html[at:]
    at = html.rindex("</body>")
    with open(os.path.join(dev_dir, page), "w", encoding="utf-8", newline="") as f:
        f.write(html[:at] + footer + html[at + len("</body>"):])

if dry:
    print("Dry run, nothing sent: " + dev_dir + " (" + stamp + ")")
    sys.exit(0)

git("add", "--", "web/dev")
git("commit", "-m", "web: deploy dev pages " + stamp +
    "\n\nCo-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>", "--", "web/dev")
changed = git("show", "--name-only", "--format=", "HEAD").splitlines()
outside = [p for p in changed if not p.startswith("web/dev/")]
if outside:
    sys.exit("Stopped before push: commit also contains " + ", ".join(outside))
git("push", "origin", "main")
print("Sent: " + stamp + " | files: " + ", ".join(changed))
print("https://tools.bulbaracing.com/dev/" + (next(iter(PAGES)) if len(PAGES) == 1 else ""))
