# Configuration file for the Sphinx documentation builder.
import sphinx_rtd_theme

# -- Project information -----------------------------------------------------
project = 'SerializedTransferProtocol_Microcontroller'
copyright = '2024, Ivan Kondratyev (Inkaros)'
author = 'Ivan Kondratyev (Inkaros)'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',  # To read doxygen-generated xml files (so that C++ documentation can be parsed)
    'sphinx_rtd_theme',  # To format the documentation html using ReadTheDocs format
]

# Breathe configuration
# Specifies the source of the C++ documentation
breathe_projects = {"SerializedTransferProtocol_Microcontroller": "./doxygen/xml"}
# Specifies default project name if C++ documentation is not available
breathe_default_project = "SerializedTransferProtocol_Microcontroller"

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # Directs sphinx to use RTD theme
