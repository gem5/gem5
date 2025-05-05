# How to build Sphinx documentation

In order to build documentation for the standard library, first install sphinx. Run these commands in the `gem5` directory.

```bash
python3 -m venv build/
source build/bin/activate
pip install sphinx

```

Next, build gem5, then cd to the `docs` directory.

```bash
scons build/ALL/gem5.opt
cd docs
```

In the `docs` directory, run `../build/ALL/gem5.opt gem5-sphinx-apidoc -o . ../src/python/gem5 -F -e` in order to generate the RST files.

To generate the documentation, run `SPHINXBUILD="../build/ALL/gem5.opt gem5-sphinx" make html`

In order to view this documentation as a website, run the following commands while still in the `docs` directory:

```bash
cd _build/html
python3 -m http.server 8000
```

If you want to use a different Sphinx theme, follow these steps:

- Install the desired theme, e.g., `pip install sphinx-rtd-theme`
- Update the `html_theme` in `docs/conf.py` by setting `html_theme = "sphinx_rtd_theme"`
- Run `SPHINXBUILD="../build/ALL/gem5.opt gem5-sphinx" make html` in the `docs` directory to apply the changes.

Note: We are aware that some modules currently display a blank page in the documentation.
This will be addressed in the future.
