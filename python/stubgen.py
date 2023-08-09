import pybind11_stubgen
import re

# Get rid of Annotated and FixedSize
# Wait for better Python support
pattern = re.compile(r"Annotated\[(List\[\w+\])\, FixedSize\([0-9]+\)\]")


def fix(match: re.Match):
    return match.group(1)


def fix_array(docstring: str):
    if docstring is None:
        return None
    return pattern.sub(fix, docstring)


if __name__ == "__main__":
    pybind11_stubgen.function_docstring_preprocessing_hooks.append(fix_array)

    pybind11_stubgen.main()
