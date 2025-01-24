#
# Copyright 2025 Hillbot Inc.
# Copyright 2020-2024 UCSD SU Lab
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import pybind11_stubgen
import re
import sys
import itertools
import types

from pybind11_stubgen import *
from pybind11_stubgen.parser.errors import InvalidIdentifierError
from pybind11_stubgen.printer import Printer
from typing import Any, Sequence
from pybind11_stubgen.structs import (
    Alias,
    Annotation,
    Argument,
    Attribute,
    Class,
    Docstring,
    Field,
    Function,
    Identifier,
    Import,
    InvalidExpression,
    Method,
    Module,
    Property,
    QualifiedName,
    ResolvedType,
    TypeVar_,
    Value,
)


# pybind11-stubgen == 2.4.2


class FixTorchJax(IParser):
    def handle_class_member(
        self, path: QualifiedName, class_: type, obj: Any
    ) -> Docstring | Alias | Class | list[Method] | Field | Property | None:
        result = super().handle_class_member(path, class_, obj)
        if not isinstance(result, list):
            return result

        for method in result:
            if str(method.function.name) == "torch":
                method.function.returns = ResolvedType(
                    QualifiedName.from_str("torch.Tensor")
                )

        for method in result:
            if str(method.function.name) == "jax":
                method.function.returns = ResolvedType(
                    QualifiedName.from_str("jax.Array")
                )

        return result

    def handle_module(
        self, path: QualifiedName, module: types.ModuleType
    ) -> Module | None:
        result = super().handle_module(path, module)
        if result is None:
            return None
        if str(path).endswith("pysapien"):
            result.imports.add(
                Import(name=None, origin=QualifiedName.from_str("torch")),
            )
            result.imports.add(
                Import(name=None, origin=QualifiedName.from_str("jax")),
            )

        return result


class FixCppFunction(IParser):
    def handle_function(self, path: QualifiedName, func: Any) -> list[Function]:
        result = super().handle_function(path, func)
        for f in result:
            if (
                isinstance(f.returns, ResolvedType)
                and isinstance(f.returns.name, QualifiedName)
                and str(f.returns.name) == "cpp_function"
            ):
                f.returns = self.parse_annotation_str("typing.Callable")
        return result


class FixNoneParameterType(IParser):
    def handle_class_member(
        self, path: QualifiedName, class_: type, obj: Any
    ) -> Docstring | Alias | Class | list[Method] | Field | Property | None:
        result = super().handle_class_member(path, class_, obj)
        if isinstance(result, list):
            for method in result:
                if isinstance(method, Method):
                    for arg in method.function.args:
                        if (
                            arg.annotation is not None
                            and isinstance(arg.annotation, ResolvedType)
                            and arg.default is not None
                            and isinstance(arg.default, Value)
                            and str(arg.default) == "None"
                        ):
                            arg.annotation = self.parse_annotation_str(
                                f"typing.Optional[{str(arg.annotation)}]"
                            )

        return result


class FixAutoListToArrayConversion(IParser):
    def handle_class_member(
        self, path: QualifiedName, class_: type, obj: Any
    ) -> Docstring | Alias | Class | list[Method] | Field | Property | None:
        result = super().handle_class_member(path, class_, obj)
        if isinstance(result, list):
            for method in result:
                if isinstance(method, Method):
                    for arg in method.function.args:
                        if (
                            arg.annotation is not None
                            and isinstance(arg.annotation, ResolvedType)
                            and str(arg.annotation.name) == "numpy.ndarray"
                            and arg.annotation.parameters is not None
                            and len(arg.annotation.parameters) >= 2
                            and isinstance(arg.annotation.parameters[1], ResolvedType)
                            and str(arg.annotation.parameters[1].name) == "numpy.dtype"
                        ):
                            list_type = ResolvedType(QualifiedName.from_str("list"))
                            params = arg.annotation.parameters[1].parameters

                            # figure out list[int] or list[float]
                            if (
                                isinstance(arg.annotation.parameters[0], ResolvedType)
                                and str(arg.annotation.parameters[0].name)
                                == "typing.Literal"
                                and params is not None
                                and isinstance(params[0], ResolvedType)
                            ):
                                if "float" in str(params[0].name):
                                    list_type = self.parse_annotation_str("list[float]")
                                elif "int" in str(params[0].name):
                                    list_type = self.parse_annotation_str("list[int]")

                            arg.annotation = ResolvedType(
                                QualifiedName.from_str("typing.Union"),
                                [
                                    arg.annotation,
                                    list_type,
                                    ResolvedType(QualifiedName.from_str("tuple")),
                                ],
                            )
        return result


class FixFindComponentByType(IParser):
    def handle_class_member(
        self, path: QualifiedName, class_: type, obj: Any
    ) -> Docstring | Alias | Class | list[Method] | Field | Property | None:
        result = super().handle_class_member(path, class_, obj)
        if not isinstance(result, list):
            return result

        for method in result:
            if str(method.function.name) == "find_component_by_type":
                method.function.returns = ResolvedType(QualifiedName.from_str("_T"))
                assert len(method.function.args) == 2
                method.function.args[1].annotation = ResolvedType(
                    QualifiedName.from_str("typing.Type"),
                    [ResolvedType(QualifiedName.from_str("_T"))],
                )

        return result

    def handle_module(
        self, path: QualifiedName, module: types.ModuleType
    ) -> Module | None:
        result = super().handle_module(path, module)
        if result is None:
            return None
        if str(path).endswith("pysapien"):
            result.imports.add(
                Import(name=None, origin=QualifiedName.from_str("typing")),
            )
            result.type_vars.append(
                TypeVar_(
                    name=Identifier("_T"),
                    constraints=[ResolvedType((QualifiedName.from_str("Component")))],
                ),
            )

        return result


def stub_parser() -> IParser:
    error_handlers_top: list[type] = [
        LoggerData,
        # IgnoreAllErrors,
    ]
    error_handlers_bottom: list[type] = [
        LogErrors,
        SuggestCxxSignatureFix,
    ]

    class Parser(
        *error_handlers_top,  # type: ignore[misc]
        FixMissing__future__AnnotationsImport,
        FixMissing__all__Attribute,
        FixMissingNoneHashFieldAnnotation,
        FixMissingImports,
        FilterTypingModuleAttributes,
        FixPEP585CollectionNames,
        FixTorchJax,
        FixCppFunction,
        FixNoneParameterType,
        FixTypingTypeNames,
        FixScipyTypeArguments,
        FixMissingFixedSizeImport,
        FixMissingEnumMembersAnnotation,
        OverridePrintSafeValues,
        FixNumpyArrayDimTypeVar,
        FixNumpyDtype,
        FixNumpyArrayFlags,
        FixFindComponentByType,
        FixCurrentModulePrefixInTypeNames,
        FixBuiltinTypes,
        RewritePybind11EnumValueRepr,
        FilterClassMembers,
        FixAutoListToArrayConversion,
        ReplaceReadWritePropertyWithField,
        FilterInvalidIdentifiers,
        FixValueReprRandomAddress,
        FixRedundantBuiltinsAnnotation,
        FilterPybindInternals,
        FixRedundantMethodsFromBuiltinObject,
        RemoveSelfAnnotation,
        FixPybind11EnumStrDoc,
        ExtractSignaturesFromPybind11Docstrings,
        ParserDispatchMixin,
        BaseParser,
        *error_handlers_bottom,  # type: ignore[misc]
    ):
        pass

    parser = Parser()

    return parser


run(
    parser=stub_parser(),
    printer=Printer(invalid_expr_as_ellipses=True),
    module_name="sapien",
    out_dir=Path("stubs"),
    sub_dir=None,
    dry_run=False,
    writer=Writer(),
)
