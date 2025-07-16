import os
import re

PACKAGE_NAME = 'ros2_topic'
TARGET_DIR = os.path.join(PACKAGE_NAME)
SETUP_FILE = 'setup.py'

def generate_entry_points():
    files = os.listdir(TARGET_DIR)
    entry_points = []

    for f in files:
        if f.endswith('.py') and f != '__init__.py':
            module_name = f[:-3]  # remove .py
            entry = f"{module_name} = {PACKAGE_NAME}.{module_name}:main"
            entry_points.append(entry)

    return entry_points

def update_setup_py(entry_points):
    with open(SETUP_FILE, 'r') as f:
        content = f.read()

    # Replace the existing console_scripts block
    new_entry = '\n            '.join([f"'{e}'," for e in entry_points])
    new_entry_block = f"""'console_scripts': [
            {new_entry}
        ],"""

    updated_content = re.sub(
        r"'console_scripts': \[.*?\],",
        new_entry_block,
        content,
        flags=re.DOTALL
    )

    with open(SETUP_FILE, 'w') as f:
        f.write(updated_content)

    print("✅ setup.py updated with these entry points:")
    for e in entry_points:
        print(f"  - {e}")

if __name__ == "__main__":
    entries = generate_entry_points()
    update_setup_py(entries)
