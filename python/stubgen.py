import pybind11_stubgen
import re
import sys
import itertools

from pybind11_stubgen import (
    ModuleStubsGenerator as _ModuleStubsGenerator,
    BARE_NUPMY_NDARRAY,
)

# TODO: ensure version "pybind11-stubgen==0.16.1"


# HACK: get a TypeVar for better annotation
class ModuleStubsGenerator(_ModuleStubsGenerator):
    def to_lines(self):
        result = []

        if self.doc_string:
            result += ['"""' + self.doc_string.replace('"""', r"\"\"\"") + '"""']

        if sys.version_info[:2] >= (3, 7):
            result += ["from __future__ import annotations"]

        result += ["import {}".format(self.module.__name__)]

        # import everything from typing
        result += ["import typing"]

        for name, class_ in self.imported_classes.items():
            class_name = getattr(class_, "__qualname__", class_.__name__)
            if name == class_name:
                suffix = ""
            else:
                suffix = " as {}".format(name)
            result += [
                "from {} import {}{}".format(class_.__module__, class_name, suffix)
            ]

        # import used packages
        used_modules = sorted(self.get_involved_modules_names())
        if used_modules:
            # result.append("if TYPE_CHECKING:")
            # result.extend(map(self.indent, map(lambda m: "import {}".format(m), used_modules)))
            result.extend(map(lambda mod: "import {}".format(mod), used_modules))

        if "numpy" in used_modules and not BARE_NUPMY_NDARRAY:
            result += ["_Shape = typing.Tuple[int, ...]"]
            result += ['_T = typing.TypeVar("T")']

        # add space between imports and rest of module
        result += [""]

        globals_ = {}
        exec("from {} import *".format(self.module.__name__), globals_)
        all_ = set(member for member in globals_.keys() if member.isidentifier()) - {
            "__builtins__"
        }
        result.append(
            "__all__ = [\n    "
            + ",\n    ".join(map(lambda s: '"%s"' % s, sorted(all_)))
            + "\n]\n\n"
        )

        for x in itertools.chain(
            self.classes, self.free_functions, self.attributes, self.alias
        ):
            result.extend(x.to_lines())
        result.append("")  # Newline at EOF
        return result


pybind11_stubgen.ModuleStubsGenerator = ModuleStubsGenerator


def fix(match: re.Match):
    return match.group(1)


def fix_array(docstring: str):
    if docstring is None:
        return None

    # manual signature for find_component_by_type
    if "find_component_by_type" in docstring:
        return "find_component_by_type(self: sapien.pysapien.Entity, cls: typing.Type[_T]) -> _T"

    # Get rid of Annotated and FixedSize
    return re.sub(
        r"Annotated\[(list\[\w+\])\,\ FixedSize\([0-9]+\)\]", r"\1", docstring
    )


if __name__ == "__main__":
    pybind11_stubgen.function_docstring_preprocessing_hooks.append(fix_array)

    pybind11_stubgen.main()
