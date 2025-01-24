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
import hashlib
from typing import List, Union
import os
import json


def get_file_md5(files: Union[str, List[str]]):
    if isinstance(files, str):
        files = [files]
    md5 = hashlib.md5()
    for filename in files:
        with open(filename, "rb") as f:
            md5.update(f.read())
    return md5.hexdigest()


def file_exists(files: Union[str, List[str]]):
    if isinstance(files, str):
        files = [files]
    return all(os.path.exists(f) for f in files)


def serialize(input_file, input_md5, output_file, output_md5, param_md5):
    return {
        "version": 0,
        "input": {"file": input_file, "md5": input_md5},
        "output": {"file": output_file, "md5": output_md5},
        "params": {"md5": param_md5},
    }


def read_hash_file(filename):
    try:
        with open(filename, "r") as f:
            return json.load(f)
    except json.JSONDecodeError:
        return {}


def write_hash_file(
    filename, input_file, input_md5, output_file, output_md5, param_md5
):
    with open(filename, "w") as f:
        json.dump(
            serialize(input_file, input_md5, output_file, output_md5, param_md5),
            f,
        )


def check_hash(hash_file, input_file, input_md5, output_file, output_md5, param_md5):
    return read_hash_file(hash_file) == serialize(
        input_file,
        input_md5,
        output_file,
        output_md5,
        param_md5,
    )


def cached(checksum_suffix):
    def wrapper(process_func):
        def func(input_file, output_file, **kwargs):
            input_file = os.path.abspath(input_file)
            output_file = os.path.abspath(output_file)

            if not os.path.exists(input_file):
                raise FileNotFoundError(input_file)

            param_md5 = hashlib.md5()
            try:
                jsonargs = json.dumps(kwargs, sort_keys=True).encode()
            except Exception as e:
                raise RuntimeError("args cannot be encoded by json") from e
            param_md5.update(jsonargs)
            param_md5 = param_md5.hexdigest()

            input_md5 = get_file_md5(input_file)

            # cache hit
            hash_file = input_file + checksum_suffix
            if os.path.exists(hash_file) and os.path.exists(output_file):
                output_md5 = get_file_md5(output_file)
                if check_hash(
                    hash_file, input_file, input_md5, output_file, output_md5, param_md5
                ):
                    return

            process_func(input_file, output_file, **kwargs)

            if not os.path.exists(output_file):
                raise FileNotFoundError("failed to generate " + output_file)

            output_md5 = get_file_md5(output_file)

            write_hash_file(
                hash_file, input_file, input_md5, output_file, output_md5, param_md5
            )

        return func

    return wrapper
