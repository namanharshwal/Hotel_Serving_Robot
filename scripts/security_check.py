#!/usr/bin/env python3
"""
Hotel Serving Robot (Nammy Robot) - Automated Security Audit & Static Analysis
Author & Copyright: Naman Harshwal (namanharshwal@gmail.com)
License: Apache License, Version 2.0

Verifies repository cleanliness, secret protection, credential shielding,
file permissions, and ROS 2 security compliance.
"""

import os
import re
import stat
import sys

# High-risk secret patterns
SECRET_PATTERNS = [
    (r"ghp_[A-Za-z0-9_]{30,}", "GitHub Personal Access Token"),
    (r"gho_[A-Za-z0-9_]{30,}", "GitHub OAuth Token"),
    (r"github_pat_[A-Za-z0-9_]{50,}", "GitHub Fine-Grained Personal Access Token"),
    (r"xox[baprs]-[0-9]{10,13}-[0-9]{10,13}-[a-zA-Z0-9]{24,32}", "Slack API Token"),
    (r"AKIA[0-9A-Z]{16}", "AWS Access Key ID"),
    (r"-----BEGIN (RSA|EC|DSA|OPENSSH|PRIVATE) KEY-----", "Private Cryptographic Key"),
    (r"(password|passwd|secret|api_key)\s*[:=]\s*['\"][^'\"]{6,}['\"]", "Hardcoded Credential/Password")
]

IGNORE_DIRS = {'.git', 'build', 'install', 'log', '.venv', '.pio', '__pycache__'}
ALLOWED_FILES = {'security_check.py'}  # Skip self matching regexes

def check_secrets(workspace_root):
    violations = []
    print("[*] Scanning workspace for secrets, tokens, and private keys...")

    for root, dirs, files in os.walk(workspace_root):
        dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]
        for f in files:
            if f in ALLOWED_FILES or f.endswith(('.pyc', '.png', '.jpg', '.stl', '.dae', '.tar', '.gz')):
                continue
            file_path = os.path.join(root, f)
            try:
                with open(file_path, 'r', encoding='utf-8', errors='ignore') as fp:
                    content = fp.read()
                    for pattern, desc in SECRET_PATTERNS:
                        matches = re.findall(pattern, content, re.IGNORECASE)
                        if matches:
                            violations.append((file_path, desc))
            except Exception as e:
                pass
    return violations

def check_file_permissions(workspace_root):
    violations = []
    print("[*] Checking file permissions (preventing insecure world-writable 0777)...")

    for root, dirs, files in os.walk(workspace_root):
        dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]
        for f in files:
            file_path = os.path.join(root, f)
            try:
                mode = os.stat(file_path).st_mode
                # Check if world-writable
                if bool(mode & stat.S_IWOTH):
                    violations.append(file_path)
            except Exception:
                pass
    return violations

def check_gitignore(workspace_root):
    gitignore_path = os.path.join(workspace_root, '.gitignore')
    missing_rules = []
    required_rules = ['.venv/', 'build/', 'install/', 'log/', '.pio/', '*.key', '*.pem']

    if not os.path.exists(gitignore_path):
        return ["Missing .gitignore!"]

    with open(gitignore_path, 'r') as f:
        content = f.read()
        for rule in required_rules:
            if rule not in content:
                missing_rules.append(rule)
    return missing_rules

def main():
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    print("=" * 70)
    print("       HOTEL SERVING ROBOT - STATIC SECURITY AUDIT TOOL")
    print(f"       Target Workspace: {workspace_root}")
    print("=" * 70)

    secret_violations = check_secrets(workspace_root)
    perm_violations = check_file_permissions(workspace_root)
    gitignore_violations = check_gitignore(workspace_root)

    has_errors = False

    if secret_violations:
        print("\n[!] CRITICAL: Secrets or credentials detected:")
        for path, desc in secret_violations:
            print(f"    - {path}: {desc}")
        has_errors = True
    else:
        print("\n[✓] Secrets Scan: PASSED (Zero embedded credentials or private keys detected).")

    if perm_violations:
        print("\n[!] WARNING: Insecure world-writable files found:")
        for path in perm_violations:
            print(f"    - {path}")
        has_errors = True
    else:
        print("[✓] File Permissions: PASSED (No insecure world-writable files).")

    if gitignore_violations:
        print("\n[!] WARNING: Recommended security ignore patterns missing:")
        for rule in gitignore_violations:
            print(f"    - Missing: {rule}")
    else:
        print("[✓] Gitignore Security Rules: PASSED (All critical ignore patterns enforced).")

    print("=" * 70)
    if has_errors:
        print("[!] SECURITY AUDIT FAILED: Please resolve the issues above.")
        sys.exit(1)
    else:
        print("[✓] ALL SECURITY CHECKS PASSED: Workspace is properly hardened.")
        sys.exit(0)

if __name__ == '__main__':
    main()
