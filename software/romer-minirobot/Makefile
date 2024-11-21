SHELL := /bin/bash

build:
	python3 -m build

install:
	python3 -m pip install ./dist/remin-*.tar.gz

ebuild:
	python3 -m pip install -e .

uninstall:
	python3 -m pip uninstall romer_minirobot

upload:
	python3 -m twine upload dist/*

tree:
	tree --gitignore .

yapf:
	yapf --in-place --recursive ./src

pylint:
	pylint ./src