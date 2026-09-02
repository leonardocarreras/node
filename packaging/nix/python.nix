# SPDX-FileCopyrightText: 2023 OPAL-RT Germany GmbH
# SPDX-License-Identifier: Apache-2.0
{
  src,
  pkgs,
  python3Packages,
  villas,
}:
python3Packages.buildPythonPackage {
  name = "villas-node";
  inherit src;
  format = "pyproject";
  dontUseCmakeConfigure = true;
  nativeBuildInputs = villas.nativeBuildInputs ++ [ python3Packages.pybind11 ];
  buildInputs = villas.buildInputs;
  propagatedBuildInputs = with python3Packages; [
    linuxfd
    requests
    protobuf
  ];
  build-system = with python3Packages; [
    scikit-build-core
    pybind11
  ];
  nativeCheckInputs = with python3Packages; [
    black
    flake8
    mypy
    pytest
    types-requests
    types-protobuf
    mypy-protobuf

    pytestCheckHook
  ];

  postPatch = ''
    ${pkgs.protobuf}/bin/protoc --proto_path ${src}/lib/formats --mypy_out=python/villas/node --python_out=python/villas/node/ ${src}/lib/formats/villas.proto
  '';
}
