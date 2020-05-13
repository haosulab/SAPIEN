rm -rf dist
python3 setup.py bdist_wheel
delocate-wheel -v dist/*.whl
twine upload dist/*.whl