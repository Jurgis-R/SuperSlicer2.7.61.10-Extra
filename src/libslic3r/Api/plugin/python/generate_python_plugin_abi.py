#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Generate the C ABI manifest required by one Python plugin.

The analyzer never imports the plugin. It parses the shared Python bindings to
associate their public functions and method names with C headers, then parses
the plugin and its local imports. Calls whose receiver type is unknown use the
union of matching binding method contracts; this may be conservative but cannot
silently omit an ABI requirement.
"""

from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass, field
import json
from pathlib import Path
import sys


DECORATOR_NAME = "abi_requires_headers"
MANDATORY_HEADER = "slic3r_plugin_types.h"


class AbiAnalysisError(ValueError):
    """Report one source location whose ABI use cannot be represented safely."""

    def __init__(self, path: Path, node: ast.AST, message: str):
        line = getattr(node, "lineno", 1)
        column = getattr(node, "col_offset", 0) + 1
        super().__init__(f"{path}:{line}:{column}: {message}")


@dataclass
class AbiCatalog:
    headers: dict[str, dict]
    c_functions: dict[str, tuple[str, ...]]
    python_symbols: dict[str, tuple[str, ...]]

    @classmethod
    def load(cls, path: Path) -> "AbiCatalog":
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"Cannot read ABI catalogue '{path}': {error}") from error
        if data.get("format_version") != 1:
            raise ValueError(f"Unsupported ABI catalogue format in '{path}'")
        headers = data.get("headers")
        functions = data.get("c_functions")
        symbols = data.get("python_symbols")
        if not isinstance(headers, dict) or not isinstance(functions, dict) or not isinstance(symbols, dict):
            raise ValueError(f"Malformed ABI catalogue '{path}'")
        return cls(
            headers=headers,
            c_functions={name: tuple(owners) for name, owners in functions.items()},
            python_symbols={name: tuple(owners) for name, owners in symbols.items()},
        )

    def validate_headers(self, headers, path: Path, node: ast.AST) -> set[str]:
        result = set()
        for header in headers:
            if header not in self.headers:
                raise AbiAnalysisError(path, node, f"unknown C ABI header '{header}'")
            result.add(header)
        return result

    def close_requirements(self, headers) -> set[str]:
        result = {MANDATORY_HEADER}
        pending = list(headers)
        while pending:
            header = pending.pop()
            if header in result:
                continue
            if header not in self.headers:
                raise ValueError(f"Unknown C ABI header '{header}'")
            result.add(header)
            pending.extend(self.headers[header].get("dependencies", ()))
        if MANDATORY_HEADER not in self.headers:
            raise ValueError(f"ABI catalogue lacks mandatory header '{MANDATORY_HEADER}'")
        return result


@dataclass
class CallableInfo:
    name: str
    simple_name: str
    module_name: str
    path: Path
    node: ast.FunctionDef | ast.AsyncFunctionDef
    owner_class: str | None = None
    requirements: set[str] = field(default_factory=set)
    called_names: set[str] = field(default_factory=set)
    called_self_methods: set[str] = field(default_factory=set)


@dataclass
class BindingIndex:
    modules: set[str] = field(default_factory=set)
    symbols: dict[str, set[str]] = field(default_factory=dict)
    methods: dict[str, set[str]] = field(default_factory=dict)
    classes: set[str] = field(default_factory=set)
    class_methods: dict[tuple[str, str], set[str]] = field(default_factory=dict)
    class_bases: dict[str, tuple[str, ...]] = field(default_factory=dict)
    method_returns: dict[tuple[str, str], str] = field(default_factory=dict)
    method_items: dict[tuple[str, str], str] = field(default_factory=dict)

    def method_requirements(self, class_name: str, method_name: str) -> set[str] | None:
        """Return one receiver class' method requirements, including inherited methods."""
        pending = [class_name]
        visited = set()
        while pending:
            current = pending.pop(0)
            if current in visited:
                continue
            visited.add(current)
            requirements = self.class_methods.get((current, method_name))
            if requirements is not None:
                return requirements
            pending.extend(self.class_bases.get(current, ()))
        return None

    def method_return(self, class_name: str, method_name: str) -> str | None:
        """Return the annotated binding class produced by one receiver method."""
        pending = [class_name]
        visited = set()
        while pending:
            current = pending.pop(0)
            if current in visited:
                continue
            visited.add(current)
            result = self.method_returns.get((current, method_name))
            if result is not None:
                return result
            pending.extend(self.class_bases.get(current, ()))
        return None

    def method_item(self, class_name: str, method_name: str) -> str | None:
        """Return the binding class yielded by an annotated receiver method."""
        pending = [class_name]
        visited = set()
        while pending:
            current = pending.pop(0)
            if current in visited:
                continue
            visited.add(current)
            result = self.method_items.get((current, method_name))
            if result is not None:
                return result
            pending.extend(self.class_bases.get(current, ()))
        return None


def attribute_parts(node: ast.AST) -> list[str] | None:
    """Return a dotted attribute path when it has a simple name as its root."""
    parts = []
    current = node
    while isinstance(current, ast.Attribute):
        parts.append(current.attr)
        current = current.value
    if not isinstance(current, ast.Name):
        return None
    parts.append(current.id)
    return list(reversed(parts))


def annotation_name(node: ast.AST | None) -> str | None:
    """Extract the principal class name from a simple or forward annotation."""
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        parts = attribute_parts(node)
        return parts[-1] if parts else None
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        try:
            return annotation_name(ast.parse(node.value, mode="eval").body)
        except SyntaxError:
            return None
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.BitOr):
        return annotation_name(node.left) or annotation_name(node.right)
    if isinstance(node, ast.Subscript):
        container = annotation_name(node.value)
        if container in {"Optional", "Union"}:
            return annotation_name(node.slice)
    return None


def annotation_item_name(node: ast.AST | None) -> str | None:
    """Extract ``T`` from an iterable return annotation such as ``Iterator[T]``."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        try:
            return annotation_item_name(ast.parse(node.value, mode="eval").body)
        except SyntaxError:
            return None
    if not isinstance(node, ast.Subscript):
        return None
    container = annotation_name(node.value)
    if container not in {"Iterable", "Iterator", "Sequence", "list", "set"}:
        return None
    return annotation_name(node.slice)


def decorator_headers(path: Path, node: ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef,
                      catalog: AbiCatalog) -> set[str]:
    """Read literal header arguments from decorators without executing Python."""
    headers = []
    for decorator in node.decorator_list:
        if not isinstance(decorator, ast.Call):
            continue
        parts = attribute_parts(decorator.func)
        if not parts or parts[-1] != DECORATOR_NAME:
            continue
        if decorator.keywords:
            raise AbiAnalysisError(path, decorator, f"{DECORATOR_NAME} accepts positional header names only")
        for argument in decorator.args:
            if not isinstance(argument, ast.Constant) or not isinstance(argument.value, str):
                raise AbiAnalysisError(path, argument, f"{DECORATOR_NAME} header names must be string literals")
            normalized = argument.value.strip().replace("\\", "/")
            if not normalized:
                raise AbiAnalysisError(path, argument, "ABI header names cannot be empty")
            headers.append(normalized)
        if not decorator.args:
            raise AbiAnalysisError(path, decorator, f"{DECORATOR_NAME} requires at least one header")
    return catalog.validate_headers(headers, path, node)


class CallableUseVisitor(ast.NodeVisitor):
    """Collect direct C calls, generated symbols and calls to other bindings."""

    def __init__(self, path: Path, catalog: AbiCatalog, class_headers: set[str]):
        self.path = path
        self.catalog = catalog
        self.requirements = set(class_headers)
        self.called_names = set()
        self.called_self_methods = set()

    def visit_Name(self, node: ast.Name) -> None:
        self.requirements.update(self.catalog.python_symbols.get(node.id, ()))

    def visit_Attribute(self, node: ast.Attribute) -> None:
        parts = attribute_parts(node)
        if parts and "host" in parts:
            host_idx = len(parts) - 2
            if host_idx >= 0 and parts[host_idx] == "host":
                function_name = parts[-1]
                owners = self.catalog.c_functions.get(function_name)
                if owners is None:
                    raise AbiAnalysisError(self.path, node, f"unknown C host function '{function_name}'")
                self.requirements.update(owners)
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call) -> None:
        if isinstance(node.func, ast.Name):
            self.called_names.add(node.func.id)
        elif isinstance(node.func, ast.Attribute):
            parts = attribute_parts(node.func)
            if parts and len(parts) == 2 and parts[0] == "self":
                self.called_self_methods.add(parts[-1])
        self.generic_visit(node)


def runtime_source_paths(runtime_root: Path) -> list[Path]:
    """Return only hand-written binding modules, excluding generators and tests."""
    excluded = {"generate_slic3r_api.py", "generate_python_plugin_abi.py", "slic3r_api_generated.py"}
    return sorted(path for path in runtime_root.rglob("*.py") if path.name not in excluded)


def build_binding_index(runtime_root: Path, catalog: AbiCatalog) -> BindingIndex:
    """Infer transitive ABI requirements for shared Python binding callables."""
    callables = []
    classes = {}
    class_bases = {}
    symbol_aliases = {}
    module_names = set()

    for path in runtime_source_paths(runtime_root):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        module_name = path.relative_to(runtime_root).with_suffix("").as_posix().replace("/", ".")
        if module_name.endswith(".__init__"):
            module_name = module_name[:-9]
        module_names.add(module_name)
        for item in tree.body:
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                visitor = CallableUseVisitor(path, catalog, set())
                visitor.visit(item)
                info = CallableInfo(
                    f"{module_name}.{item.name}", item.name, module_name, path, item, None,
                    decorator_headers(path, item, catalog) | visitor.requirements,
                    visitor.called_names, visitor.called_self_methods)
                callables.append(info)
            elif isinstance(item, ast.ClassDef):
                class_headers = decorator_headers(path, item, catalog)
                classes[item.name] = set(class_headers)
                class_bases[item.name] = tuple(
                    name for name in (annotation_name(base) for base in item.bases) if name
                )
                for member in item.body:
                    if not isinstance(member, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        continue
                    visitor = CallableUseVisitor(path, catalog, class_headers)
                    visitor.visit(member)
                    info = CallableInfo(
                        f"{module_name}.{item.name}.{member.name}", member.name, module_name,
                        path, member, item.name,
                        decorator_headers(path, member, catalog) | visitor.requirements,
                        visitor.called_names, visitor.called_self_methods)
                    callables.append(info)
            elif isinstance(item, (ast.Assign, ast.AnnAssign)):
                value = item.value
                targets = item.targets if isinstance(item, ast.Assign) else [item.target]
                if isinstance(value, ast.Name):
                    for target in targets:
                        if isinstance(target, ast.Name):
                            symbol_aliases[target.id] = value.id

    top_level_by_simple = {}
    top_level_by_module = {}
    by_method = {}
    by_class_method = {}
    for info in callables:
        if info.owner_class is None:
            top_level_by_simple.setdefault(info.simple_name, []).append(info)
            top_level_by_module[(info.module_name, info.simple_name)] = info
        else:
            by_method.setdefault(info.simple_name, []).append(info)
            by_class_method[(info.owner_class, info.simple_name)] = info

    changed = True
    while changed:
        changed = False
        for info in callables:
            expanded = set(info.requirements)
            for name in info.called_names:
                local = top_level_by_module.get((info.module_name, name))
                if local is not None:
                    expanded.update(local.requirements)
                else:
                    for called in top_level_by_simple.get(name, ()):
                        expanded.update(called.requirements)
            if info.owner_class is not None:
                for name in info.called_self_methods:
                    called = by_class_method.get((info.owner_class, name))
                    if called is not None:
                        expanded.update(called.requirements)
            if expanded != info.requirements:
                info.requirements = expanded
                changed = True

    index = BindingIndex(
        modules=module_names,
        classes=set(classes),
        class_bases=class_bases,
    )
    for info in callables:
        if info.owner_class is None:
            index.symbols.setdefault(info.simple_name, set()).update(info.requirements)
        # Class methods are receiver operations, not importable module symbols.
        # Dunder methods are implementation details: a known class constructor
        # is accounted for below through the class symbol and its own __init__,
        # without unioning every constructor into ``value.__init__()``.
        else:
            index.class_methods[(info.owner_class, info.simple_name)] = set(info.requirements)
            return_name = annotation_name(info.node.returns)
            if return_name in classes:
                index.method_returns[(info.owner_class, info.simple_name)] = return_name
            item_name = annotation_item_name(info.node.returns)
            if item_name in classes:
                index.method_items[(info.owner_class, info.simple_name)] = item_name
            if not info.simple_name.startswith("__"):
                index.methods.setdefault(info.simple_name, set()).update(info.requirements)
    for info in callables:
        if info.module_name == "slic3r_api" and info.owner_class is None:
            index.symbols[info.simple_name] = set(info.requirements)
    for class_name, requirements in classes.items():
        index.symbols.setdefault(class_name, set()).update(requirements)
        init = by_class_method.get((class_name, "__init__"))
        if init is not None:
            index.symbols[class_name].update(init.requirements)
    changed = True
    while changed:
        changed = False
        for alias, source in symbol_aliases.items():
            requirements = set(index.symbols.get(source, ()))
            requirements.update(catalog.python_symbols.get(source, ()))
            if requirements and not requirements.issubset(index.symbols.setdefault(alias, set())):
                index.symbols[alias].update(requirements)
                changed = True
    return index


class PluginUseVisitor(ast.NodeVisitor):
    """Collect requirements from one plugin module and reject unsafe dynamic API use."""

    def __init__(self, path: Path, catalog: AbiCatalog, bindings: BindingIndex):
        self.path = path
        self.catalog = catalog
        self.bindings = bindings
        self.requirements = set()
        self.runtime_imports = {}
        self.runtime_module_aliases = set()
        self.explicit_scope_stack = [False]
        self.type_scope_stack = [{}]
        self.item_scope_stack = [{}]

    def _binding_class(self, name: str | None) -> str | None:
        if name is None:
            return None
        imported = self.runtime_imports.get(name, name)
        return imported if imported in self.bindings.classes else None

    def _expression_classes(self, node: ast.AST) -> set[str]:
        if isinstance(node, ast.Name):
            for scope in reversed(self.type_scope_stack):
                if node.id in scope:
                    return scope[node.id]
            class_name = self._binding_class(node.id)
            return {class_name} if class_name is not None else set()
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name):
                class_name = self._binding_class(node.func.id)
                return {class_name} if class_name is not None else set()
            if isinstance(node.func, ast.Attribute):
                parts = attribute_parts(node.func)
                if parts and len(parts) == 2 and parts[0] in self.runtime_module_aliases:
                    class_name = self._binding_class(parts[1])
                    return {class_name} if class_name is not None else set()
                results = set()
                for receiver in self._expression_classes(node.func.value):
                    result = self.bindings.method_return(receiver, node.func.attr)
                    if result is not None:
                        results.add(result)
                return results
        return set()

    def _record_assignment_type(self, target: ast.AST, value: ast.AST) -> None:
        if not isinstance(target, ast.Name):
            return
        class_names = self._expression_classes(value)
        if class_names:
            self.type_scope_stack[-1].setdefault(target.id, set()).update(class_names)
        else:
            # An unresolved assignment must shadow any type inferred earlier.
            self.type_scope_stack[-1][target.id] = set()
        self.item_scope_stack[-1][target.id] = self._iterable_item_classes(value)

    def _iterable_item_classes(self, node: ast.AST) -> set[str]:
        if isinstance(node, ast.Name):
            for scope in reversed(self.item_scope_stack):
                if node.id in scope:
                    return scope[node.id]
        results = set()
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            for receiver in self._expression_classes(node.func.value):
                item = self.bindings.method_item(receiver, node.func.attr)
                if item is not None:
                    results.add(item)
            return results
        for receiver in self._expression_classes(node):
            item = self.bindings.method_item(receiver, "__iter__")
            if item is not None:
                results.add(item)
        return results

    def visit_Assign(self, node: ast.Assign) -> None:
        for target in node.targets:
            self._record_assignment_type(target, node.value)
        self.generic_visit(node)

    def visit_AnnAssign(self, node: ast.AnnAssign) -> None:
        if isinstance(node.target, ast.Name):
            class_name = self._binding_class(annotation_name(node.annotation))
            class_names = {class_name} if class_name is not None else set()
            if node.value is not None:
                class_names.update(self._expression_classes(node.value))
            if class_names:
                self.type_scope_stack[-1].setdefault(node.target.id, set()).update(class_names)
            else:
                self.type_scope_stack[-1][node.target.id] = set()
            item_name = annotation_item_name(node.annotation)
            item_names = {item_name} if item_name in self.bindings.classes else set()
            if node.value is not None:
                item_names.update(self._iterable_item_classes(node.value))
            self.item_scope_stack[-1][node.target.id] = item_names
        self.generic_visit(node)

    @staticmethod
    def _merge_optional_scope(before: dict[str, set[str]], after: dict[str, set[str]]) -> dict[str, set[str]]:
        """Merge a block that may execute zero times without claiming certainty."""
        merged = {}
        for name in before.keys() | after.keys():
            before_types = before.get(name)
            after_types = after.get(name)
            if not before_types or not after_types:
                merged[name] = set()
            else:
                merged[name] = before_types | after_types
        return merged

    def _visit_optional_block(self, statements: list[ast.stmt]) -> None:
        """Visit a conditional block and retain both entry and exit type states."""
        before_types = {name: set(types) for name, types in self.type_scope_stack[-1].items()}
        before_items = {name: set(types) for name, types in self.item_scope_stack[-1].items()}
        for statement in statements:
            self.visit(statement)
        self.type_scope_stack[-1] = self._merge_optional_scope(
            before_types, self.type_scope_stack[-1]
        )
        self.item_scope_stack[-1] = self._merge_optional_scope(
            before_items, self.item_scope_stack[-1]
        )

    def visit_For(self, node: ast.For) -> None:
        self.visit(node.iter)
        item_types = self._iterable_item_classes(node.iter)
        before_types = {name: set(types) for name, types in self.type_scope_stack[-1].items()}
        before_items = {name: set(types) for name, types in self.item_scope_stack[-1].items()}
        if isinstance(node.target, ast.Name):
            self.type_scope_stack[-1][node.target.id] = item_types
            self.item_scope_stack[-1][node.target.id] = set()
        else:
            self.visit(node.target)
        for statement in node.body:
            self.visit(statement)
        self.type_scope_stack[-1] = self._merge_optional_scope(
            before_types, self.type_scope_stack[-1]
        )
        self.item_scope_stack[-1] = self._merge_optional_scope(
            before_items, self.item_scope_stack[-1]
        )
        self._visit_optional_block(node.orelse)

    def visit_AsyncFor(self, node: ast.AsyncFor) -> None:
        self.visit_For(node)

    def _visit_comprehension(self, generators: list[ast.comprehension], outputs: list[ast.AST]) -> None:
        """Visit a comprehension in evaluation order using its isolated scope."""
        self.type_scope_stack.append({})
        self.item_scope_stack.append({})
        for generator in generators:
            self.visit(generator.iter)
            item_types = self._iterable_item_classes(generator.iter)
            if isinstance(generator.target, ast.Name):
                self.type_scope_stack[-1][generator.target.id] = item_types
                self.item_scope_stack[-1][generator.target.id] = set()
            else:
                self.visit(generator.target)
            for condition in generator.ifs:
                self.visit(condition)
        for output in outputs:
            self.visit(output)
        self.item_scope_stack.pop()
        self.type_scope_stack.pop()

    def visit_ListComp(self, node: ast.ListComp) -> None:
        self._visit_comprehension(node.generators, [node.elt])

    def visit_SetComp(self, node: ast.SetComp) -> None:
        self._visit_comprehension(node.generators, [node.elt])

    def visit_DictComp(self, node: ast.DictComp) -> None:
        self._visit_comprehension(node.generators, [node.key, node.value])

    def visit_GeneratorExp(self, node: ast.GeneratorExp) -> None:
        self._visit_comprehension(node.generators, [node.elt])

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        if node.module in self.bindings.modules:
            for alias in node.names:
                if alias.name == "*":
                    raise AbiAnalysisError(self.path, alias, "wildcard imports from the plugin API cannot be analyzed safely")
                known = (alias.name == DECORATOR_NAME or
                         alias.name in self.bindings.symbols or
                         alias.name in self.catalog.python_symbols)
                if not known:
                    raise AbiAnalysisError(
                        self.path, alias,
                        f"unknown Python plugin API symbol '{alias.name}' in module '{node.module}'")
                self.runtime_imports[alias.asname or alias.name] = alias.name
        self.generic_visit(node)

    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            if alias.name in self.bindings.modules:
                self.runtime_module_aliases.add(alias.asname or alias.name)

    def visit_Name(self, node: ast.Name) -> None:
        imported = self.runtime_imports.get(node.id)
        if imported is not None:
            self.requirements.update(self.bindings.symbols.get(imported, ()))
            self.requirements.update(self.catalog.python_symbols.get(imported, ()))

    def visit_Attribute(self, node: ast.Attribute) -> None:
        parts = attribute_parts(node)
        if parts:
            if parts[0] in self.runtime_module_aliases and len(parts) == 2:
                requirements = set(self.bindings.symbols.get(parts[1], ()))
                requirements.update(self.catalog.python_symbols.get(parts[1], ()))
                known = (parts[1] == DECORATOR_NAME or
                         parts[1] in self.bindings.symbols or
                         parts[1] in self.catalog.python_symbols)
                if not known:
                    raise AbiAnalysisError(
                        self.path, node, f"unknown Python plugin API symbol '{parts[1]}'")
                self.requirements.update(requirements)
            if len(parts) >= 2 and parts[-2] == "host":
                owners = self.catalog.c_functions.get(parts[-1])
                if owners is None:
                    raise AbiAnalysisError(self.path, node, f"unknown C host function '{parts[-1]}'")
                self.requirements.update(owners)
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call) -> None:
        if isinstance(node.func, ast.Name) and node.func.id == "getattr" and node.args:
            parts = attribute_parts(node.args[0])
            if parts and (parts[-1] in {"host", "api"} or parts[0] in self.runtime_module_aliases):
                if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant) and isinstance(node.args[1].value, str):
                    name = node.args[1].value
                    if parts[-1] == "host":
                        requirements = self.catalog.c_functions.get(name)
                    else:
                        requirements = set(self.bindings.symbols.get(name, ()))
                        requirements.update(self.bindings.methods.get(name, ()))
                    if requirements:
                        self.requirements.update(requirements)
                    elif not self.explicit_scope_stack[-1]:
                        raise AbiAnalysisError(self.path, node, f"dynamic plugin API attribute '{name}' cannot be resolved")
                elif not self.explicit_scope_stack[-1]:
                    raise AbiAnalysisError(
                        self.path, node,
                        f"dynamic plugin API access requires {DECORATOR_NAME}(...) on the enclosing callable")

        if isinstance(node.func, ast.Attribute):
            parts = attribute_parts(node.func)
            if parts and not parts[-1].startswith("__") and not (len(parts) >= 2 and parts[-2] == "host"):
                receivers = self._expression_classes(node.func.value)
                requirements = set()
                completely_resolved = bool(receivers)
                for receiver in receivers:
                    receiver_requirements = self.bindings.method_requirements(receiver, parts[-1])
                    if receiver_requirements is None:
                        completely_resolved = False
                        break
                    requirements.update(receiver_requirements)
                self.requirements.update(
                    requirements if completely_resolved
                    else self.bindings.methods.get(parts[-1], ())
                )
        elif isinstance(node.func, ast.Name):
            imported = self.runtime_imports.get(node.func.id)
            if imported is not None:
                self.requirements.update(self.bindings.symbols.get(imported, ()))
        self.generic_visit(node)

    def _visit_decorated_scope(self, node) -> None:
        explicit = decorator_headers(self.path, node, self.catalog)
        self.requirements.update(explicit)
        self.explicit_scope_stack.append(self.explicit_scope_stack[-1] or bool(explicit))
        scope = {}
        item_scope = {}
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            arguments = (*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs)
            for argument in arguments:
                class_name = self._binding_class(annotation_name(argument.annotation))
                if class_name is not None:
                    scope[argument.arg] = {class_name}
                item_name = annotation_item_name(argument.annotation)
                if item_name in self.bindings.classes:
                    item_scope[argument.arg] = {item_name}
        self.type_scope_stack.append(scope)
        self.item_scope_stack.append(item_scope)
        for statement in node.body:
            self.visit(statement)
        self.item_scope_stack.pop()
        self.type_scope_stack.pop()
        self.explicit_scope_stack.pop()

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self._visit_decorated_scope(node)

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._visit_decorated_scope(node)

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        self._visit_decorated_scope(node)


def local_imports(path: Path, tree: ast.Module, source_root: Path) -> list[Path]:
    """Resolve package-local imports without consulting sys.path or importing code."""
    result = []
    source_root = source_root.resolve()
    for node in ast.walk(tree):
        candidates = []
        search_roots = [source_root, path.parent]
        if isinstance(node, ast.Import):
            candidates.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module in {"slic3r_api", "slic3r_abi"}:
                continue
            if node.level > 0:
                relative_root = path.parent
                for _unused in range(node.level - 1):
                    relative_root = relative_root.parent
                search_roots = [relative_root]
            if node.module:
                candidates.append(node.module)
            elif node.level > 0:
                candidates.extend(alias.name for alias in node.names if alias.name != "*")
        for module in candidates:
            relative = Path(*module.split("."))
            for base in search_roots:
                for candidate in (base / relative.with_suffix(".py"), base / relative / "__init__.py"):
                    resolved = candidate.resolve()
                    if candidate.is_file() and (resolved == source_root or source_root in resolved.parents) \
                            and resolved not in result:
                        result.append(resolved)
    return result


def analyze_plugin(entry: Path, source_root: Path, catalog: AbiCatalog,
                   bindings: BindingIndex) -> set[str]:
    """Analyze the entry module and every statically resolvable local import."""
    entry = entry.resolve()
    source_root = source_root.resolve()
    if entry != source_root and source_root not in entry.parents:
        raise ValueError(f"Python plugin entry '{entry}' is outside source root '{source_root}'")
    pending = [entry]
    visited = set()
    requirements = set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (OSError, SyntaxError) as error:
            raise ValueError(f"Cannot parse Python plugin source '{path}': {error}") from error
        visitor = PluginUseVisitor(path, catalog, bindings)
        visitor.visit(tree)
        requirements.update(visitor.requirements)
        pending.extend(local_imports(path, tree, source_root))
    return catalog.close_requirements(requirements)


def serialize_manifest(requirements: set[str], catalog: AbiCatalog) -> str:
    """Serialize requirements in the literal-key INI format consumed by the host."""
    lines = ["[abi]"]
    for header in sorted(requirements):
        version = catalog.headers[header]["version"]
        lines.append(f"{header} = {version[0]}.{version[1]}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate one Python plugin's C ABI manifest without executing it.")
    parser.add_argument("--entry", type=Path, required=True, help="Python plugin entry point")
    parser.add_argument("--source-root", type=Path, help="root used to resolve package-local imports")
    parser.add_argument("--runtime-root", type=Path, required=True, help="shared Python binding source directory")
    parser.add_argument("--catalog", type=Path, required=True, help="generated C symbol ABI catalogue")
    parser.add_argument("--output", type=Path, required=True, help="output ABI INI file")
    args = parser.parse_args()

    try:
        entry = args.entry.resolve()
        source_root = (args.source_root or entry.parent).resolve()
        catalog = AbiCatalog.load(args.catalog)
        bindings = build_binding_index(args.runtime_root.resolve(), catalog)
        requirements = analyze_plugin(entry, source_root, catalog, bindings)
        manifest = serialize_manifest(requirements, catalog)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(manifest, encoding="utf-8")
        temporary.replace(args.output)
        return 0
    except (AbiAnalysisError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
