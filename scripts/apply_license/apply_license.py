import os
import yaml
import re
import argparse


# Load configuration file
def load_config(config_file):
    with open(config_file, "r") as file:
        config = yaml.safe_load(file)
    return config


# Get the comment style for a file based on its extension
def get_comment_style(file_extension, comment_styles):
    return comment_styles.get(file_extension, {})


# Generate the header based on the comment style and header content
def generate_header(comment_style, header_content):
    if not comment_style:
        return ""

    comment_start = comment_style.get("start", "")
    comment_end = comment_style.get("end", "")
    comment_middle = comment_style.get("middle", "")

    header = comment_start + "\n"
    for line in header_content:
        header += comment_middle + line + "\n"
    header += comment_end + "\n"

    return header


# Check if the file already has a header
def has_existing_header(file_content, header):
    return file_content.startswith(header)


def has_copyright(file_content):
    lines = file_content.splitlines()
    header = "\n".join(lines[:20]).lower()
    return "copyright" in header or "spdx-license" in header


def has_hillbot_line(file_content):
    lines = file_content.splitlines()
    header = "\n".join(lines[:5]).lower()
    return "hillbot inc" in header


def split_skip(content: str, skip):
    skip = [re.compile(s) for s in skip]
    lines = content.splitlines()
    i = 0
    for i, l in enumerate(lines):
        if any(s.match(l) for s in skip):
            continue
        break
    head = lines[:i]
    tail = lines[i:]
    head = "\n".join(head)
    tail = "\n".join(tail)
    if head != "":
        head += "\n"
    return head, tail


# Update or add the header to the file
def update_file_header(file_path, header, comment_style):
    with open(file_path, "r") as file:
        content = file.read()
        content_header, content = split_skip(content, comment_style["skip"])

    # If the file already has a header, check if it needs to be replaced
    if has_existing_header(content, header):
        # If the header is already up-to-date, return
        if content.startswith(header):
            print("Skip:", file_path)
            return
        # Otherwise, remove the old header
        content = re.sub(
            r"^{}.*?\n".format(re.escape(comment_style.get("start", ""))), "", content, flags=re.DOTALL
        )

    if has_copyright(content):
        print("Skip 3rd party: ", file_path)
        # If there is copyright added by someone else, return
        return

    # Add the new header
    with open(file_path, "w") as file:
        print("Apply license: ", file_path)
        file.write(content_header + header + content)


# Remove the header from the file
def remove_file_header(file_path, comment_style, header_content):
    with open(file_path, "r") as file:
        content = file.read()

    if not has_hillbot_line(content):
        return

    # Generate a regex pattern to match the header
    if comment_style.get("start"):
        # Build the regex pattern to match the header
        header_pattern = re.escape(comment_style.get("start", "")) + r"\n"
        for line in header_content:
            header_pattern += re.escape(comment_style.get("middle", "")) + re.escape(line) + r"\n"
        header_pattern += re.escape(comment_style.get("end", "")) + r"\n"

        # Remove the matched header
        content = re.sub(header_pattern, "", content, flags=re.DOTALL)

    with open(file_path, "w") as file:
        file.write(content)


# Process all files in a directory
def process_directory(directory, header_content, comment_styles, delete_mode=False):
    for root, _, files in os.walk(directory):
        for file in files:
            file_path = os.path.join(root, file)
            file_extension = os.path.splitext(file)[1][1:]  # Get the file extension without the dot
            comment_style = get_comment_style(file_extension, comment_styles)
            if comment_style:
                if delete_mode:
                    remove_file_header(file_path, comment_style, header_content)
                else:
                    header = generate_header(comment_style, header_content)
                    update_file_header(file_path, header, comment_style)
            else:
                print("Skip unknown extension: ", file_path)


def main(config_file, delete_mode=False):
    config = load_config(config_file)
    header_content = config.get("header_content", [])
    comment_styles = config.get("comment_styles", {})  # lang -> style
    comment_styles = dict(
        (e, comment_styles[lang]) for lang, exts in config.get("language", {}).items() for e in exts
    )

    target_files = config.get("target_files", [])
    target_directories = config.get("target_directories", [])

    for file_path in target_files:
        file_extension = os.path.splitext(file_path)[1][1:]
        comment_style = get_comment_style(file_extension, comment_styles)
        if comment_style:
            if delete_mode:
                remove_file_header(file_path, comment_style, header_content)
            else:
                header = generate_header(comment_style, header_content)
                update_file_header(file_path, header, comment_style)

    for directory in target_directories:
        process_directory(directory, header_content, comment_styles, delete_mode)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Add or remove comment headers from source files.")
    parser.add_argument(
        "-d", "--delete", action="store_true", help="Delete the comment header instead of adding it."
    )
    parser.add_argument(
        "config_file",
        nargs="?",
        default=os.path.join(os.path.dirname(__file__), "license_config.yaml"),
        help="Path to the YAML configuration file.",
    )
    args = parser.parse_args()

    main(args.config_file, args.delete)
