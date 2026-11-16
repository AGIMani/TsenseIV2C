#!/usr/bin/env python3
"""Restore only pinned Manus runtime libraries from the customer's source ZIP."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'vendor/manussdk/runtime-manifest.json'


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def check(destination, manifest):
    for relative, record in manifest['files'].items():
        path = destination / relative
        if 'symlink' in record:
            if not path.is_symlink() or os.readlink(path) != record['symlink']:
                raise ValueError(f'Manus runtime link mismatch: {relative}')
        elif path.is_symlink() or not path.is_file() or digest(path) != record['sha256']:
            raise ValueError(f'Manus runtime missing or checksum mismatch: {relative}')


def install(archive, destination, manifest):
    # Validate every selected member before changing any installed file. Archive
    # paths and symlink targets come from the checked-in manifest, never ZIP metadata.
    with tempfile.TemporaryDirectory(prefix='manus-runtime-') as temporary:
        stage = Path(temporary)
        with zipfile.ZipFile(archive) as bundle:
            for relative, record in manifest['files'].items():
                rel = PurePosixPath(relative)
                if rel.is_absolute() or '..' in rel.parts:
                    raise ValueError('Invalid runtime manifest path')
                path = stage / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                with bundle.open(record['archive_path']) as source, path.open('wb') as output:
                    for block in iter(lambda: source.read(1024 * 1024), b''):
                        output.write(block)
                if digest(path) != record['sha256']:
                    raise ValueError(f'Customer archive checksum mismatch: {relative}')
                if 'symlink' in record:
                    target = record['symlink']
                    if PurePosixPath(target).name != target or str(rel.parent / target) not in manifest['files']:
                        raise ValueError('Invalid runtime symlink target')
                    if path.read_text() != target:
                        raise ValueError('Runtime symlink content mismatch')
        for relative, record in manifest['files'].items():
            path = destination / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            # A sibling temporary file makes replacement atomic on the target filesystem.
            with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.manus-', delete=False) as output:
                temporary_path = Path(output.name)
                if 'symlink' not in record:
                    with (stage / relative).open('rb') as source:
                        for block in iter(lambda: source.read(1024 * 1024), b''):
                            output.write(block)
            try:
                if 'symlink' in record:
                    temporary_path.unlink()
                    temporary_path.symlink_to(record['symlink'])
                else:
                    temporary_path.chmod(0o644)
                os.replace(temporary_path, path)
            finally:
                temporary_path.unlink(missing_ok=True)
    check(destination, manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', nargs='?', type=Path, help='Customer TsenseIV2C source ZIP')
    parser.add_argument('--destination', type=Path, default=ROOT / 'vendor/manussdk')
    parser.add_argument('--check', action='store_true', help='Verify files without loading SDK or accessing devices')
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    if args.check:
        check(args.destination, manifest)
    elif args.archive:
        install(args.archive, args.destination, manifest)
    else:
        parser.error('Supply the customer ZIP or --check')
    print(f'Manus {manifest["sdk_version"]} runtime verified: {args.destination}')


if __name__ == '__main__':
    main()
