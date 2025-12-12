import subprocess
import sys
from setuptools import setup
from setuptools.command.install import install

package_name = 'alicia_d_sdk'

def uninstall_existing_package():
    """Uninstall existing package if it exists."""
    try:
        subprocess.run(
            [sys.executable, "-m", "pip", "uninstall", "-y", package_name],
            capture_output=True,
            check=False
        )
    except Exception:
        pass  # Continue with installation if uninstall fails

class CustomInstallCommand(install):
    """Custom installation that uninstalls existing package first."""
    
    def run(self):
        uninstall_existing_package()
        install.run(self)

setup(
    cmdclass={
        'install': CustomInstallCommand,
    },
)