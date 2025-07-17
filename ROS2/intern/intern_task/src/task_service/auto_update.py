#!/usr/bin/env python3

import os
import re

CMAKE_FILE = "CMakeLists.txt"
SRC_DIR = "src"
MSG_DIR = "msg"
SRV_DIR = "srv"
ACTION_DIR = "action"

VALID_NAME_PATTERN = re.compile(r"^[A-Z][A-Za-z0-9]*$")

def read_cmake():
    if not os.path.exists(CMAKE_FILE):
        print(f"❌ ไม่พบ {CMAKE_FILE}")
        return []
    with open(CMAKE_FILE, "r") as f:
        return f.readlines()

def write_cmake(lines):
    with open(CMAKE_FILE, "w") as f:
        f.writelines(lines)

def ensure_find_package(lines, package):
    found = any(f"find_package({package}" in line for line in lines)
    if not found:
        print(f"🔍 เพิ่ม find_package({package})")
        insert_index = 0
        for i, line in enumerate(lines):
            if line.startswith("project("):
                insert_index = i + 1
                break
        lines.insert(insert_index, f"find_package({package} REQUIRED)\n")

def ensure_rosidl_generate_interfaces(lines, files):
    existing_block = False
    for line in lines:
        if "rosidl_generate_interfaces" in line:
            existing_block = True
            break

    block = [f'  "{f}"\n' for f in files]

    if existing_block:
        for f in files:
            if not any(f in line for line in lines):
                print(f"📦 เพิ่ม: {f}")
                for i, line in enumerate(lines):
                    if "rosidl_generate_interfaces" in line:
                        lines.insert(i + 1, f'  "{f}"\n')
                        break
    elif files:
        print("📦 สร้าง rosidl_generate_interfaces block ใหม่")
        lines.append("\nrosidl_generate_interfaces(${PROJECT_NAME}\n")
        lines.extend(block)
        lines.append(")\n")

def is_valid_interface_name(file):
    name = os.path.splitext(file)[0]
    return VALID_NAME_PATTERN.match(name) is not None

def check_invalid_interface_files(files):
    return [f for f in files if not is_valid_interface_name(os.path.basename(f))]

def update_cmake():
    lines = read_cmake()
    if not lines:
        return

    cpp_files = []
    py_files = []
    msg_files = []
    srv_files = []
    action_files = []

    if os.path.isdir(SRC_DIR):
        for file in os.listdir(SRC_DIR):
            if file.endswith(".cpp"):
                cpp_files.append(file)
            elif file.endswith(".py"):
                py_files.append(file)

    if os.path.isdir(MSG_DIR):
        msg_files = [f"msg/{file}" for file in os.listdir(MSG_DIR) if file.endswith(".msg")]
    if os.path.isdir(SRV_DIR):
        srv_files = [f"srv/{file}" for file in os.listdir(SRV_DIR) if file.endswith(".srv")]
    if os.path.isdir(ACTION_DIR):
        action_files = [f"action/{file}" for file in os.listdir(ACTION_DIR) if file.endswith(".action")]

    # 🔎 ตรวจสอบชื่อไฟล์ผิดพลาด
    all_interface_files = msg_files + srv_files + action_files
    invalid_files = check_invalid_interface_files(all_interface_files)

    if invalid_files:
        print("❌ ไม่สามารถอัปเดต CMakeLists.txt ได้ เนื่องจากชื่อไฟล์ต่อไปนี้ไม่ถูกต้อง:")
        for f in invalid_files:
            print(f"   - {f} ❌ ชื่อไม่ตรงรูปแบบ (^[A-Z][A-Za-z0-9]*$)")
        return  # หยุดการเขียนไฟล์

    # ✅ ถ้าไม่มีชื่อผิดพลาด ดำเนินต่อ

    # C++ Executables
    for cpp in cpp_files:
        exec_name = os.path.splitext(cpp)[0]
        exec_cmd = f"add_executable({exec_name} src/{cpp})\n"
        install_cmd = f"install(TARGETS {exec_name} DESTINATION lib/${{PROJECT_NAME}})\n"
        dep_cmd = f"ament_target_dependencies({exec_name} rclcpp std_msgs)\n"

        if exec_cmd not in lines:
            print(f"🛠️ เพิ่ม C++ executable: {exec_name}")
            lines.append("\n" + exec_cmd)
            lines.append(dep_cmd)
            lines.append(install_cmd)

    # Python Scripts
    python_section_found = False
    for i, line in enumerate(lines):
        if "install(PROGRAMS" in line:
            python_section_found = True
            for py in py_files:
                full_line = f"  src/{py}\n"
                if full_line not in lines[i+1:]:
                    print(f"🐍 เพิ่ม Python script: {py}")
                    lines.insert(i+1, full_line)
            break

    if not python_section_found and py_files:
        print(f"🐍 เพิ่ม install(PROGRAMS ...) section")
        lines.append("\ninstall(PROGRAMS\n")
        for py in py_files:
            lines.append(f"  src/{py}\n")
        lines.append("  DESTINATION lib/${PROJECT_NAME}\n)\n")

    # Custom Interfaces (msg, srv, action)
    if all_interface_files:
        ensure_find_package(lines, "rosidl_default_generators")
        ensure_find_package(lines, "rosidl_default_runtime")
        ensure_rosidl_generate_interfaces(lines, all_interface_files)

    write_cmake(lines)
    print("✅ อัปเดต CMakeLists.txt เรียบร้อยแล้ว!")

if __name__ == "__main__":
    update_cmake()

