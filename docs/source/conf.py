# Configuration file for the Sphinx documentation builder.

# -- Project information
import os
import sys

project = "SonoRo_swarm"
copyright = "2025, Alberto Doimo"
author = "Alberto Doimo"

# release = "0.1"
# version = "0.1.0"

# -- General configuration
sys.path.insert(
    0,
    os.path.abspath("../.."),
)


extensions = [
    "sphinx.ext.duration",
    "sphinx.ext.doctest",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
]

intersphinx_mapping = {
    "python": ("https://docs.python.org/3/", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master/", None),
}
intersphinx_disabled_domains = ["std"]

templates_path = ["_templates"]

# -- Options for HTML output

html_theme = "sphinx_rtd_theme"

# -- Options for EPUB output
epub_show_urls = "footnote"

html_logo = "_static/SonoRoSwarm_logo.jpeg"

# Hides classes and functions from the sidebar menu
toc_object_entries = False

autodoc_mock_imports = [
    "pyaudio",  # Fails on RTD (no soundcard)
    "sounddevice",  # Fails on RTD (no soundcard)
    "thymiodirect",  # Fails on RTD (no robot)
    "serial",  # Fails on RTD (no USB ports)
    "RPi",  # Fails on RTD (not a Raspberry Pi)
]
