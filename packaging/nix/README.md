# `villas` as a Nix Flake

<!--
SPDX-FileCopyrightText: 2023 OPAL-RT Germany GmbH
SPDX-License-Identifier: Apache-2.0
 -->

`VILLASnode` is also packaged as a Nix Flake.

## Setup Nix

Note that flakes are an (as of May 2023) experimental feature of the Nix
project to provide the declarative and fully reproducible builds of Nix without
the hassle of manually updating hashes.

Using `villas` as a flake thus requires the nix package manager to be installed
and the `flakes` and `nix-command` features to be enabled.

Documentation:

- Installation: <https://nixos.org/download.html>
- Enable Flakes: <https://nixos.wiki/wiki/Flakes#Enable_flakes>

Check if your installation works by running e.g.

```shell
nix run nixpkgs#hello
```

## Getting Started

Try to run `villas node` by typing

```shell
# `nix run` runs an output package provided by a flake
# `github:VILLASframework/node` is the repository containing the flake
# `?dir=packaging/nix` indicates that the `flake.nix` resides in this subdirectory
# `#villas-node-minimal` chooses the output package you want to run
# after the `--` follow the arguments to the `villas` tool (e.g. `node -h`)
nix run 'github:VILLASframework/node?dir=packaging/nix#villas-node-minimal' -- node -h
```

The [`flake.nix`] provides 2 versions of `villas`:

- `#villas-node-minimal`: `villas` CLI command without most optional dependencies
- `#villas-node`: `villas` CLI command with most optional dependencies available to Nix

The version chosen by default is `#villas-node`. Omitting the `#` suffix will
select the more complete version of the `villas` CLI command.

```shell
# Omit `#` suffix in the flake reference from above
# Note the difference supported nodes/formats in help output compared to above
nix run 'github:VILLASframework/node?dir=packaging/nix' -- node -h
```

## Simple Install

You can also install the `villas` CLI command into your local profile and have it available in
`PATH`.

```shell
nix profile install 'github:VILLASframework/node?dir=packaging/nix#villas-node'
```

If you don't want to add it directly into the global path you could add it into
the flake registry as well.

```shell
nix registry add villas-node 'github:VILLASframework/node?dir=packaging/nix'
```

This allows you to substitute all references to
`github:VILLASframework/node?dir=packaging/nix#villas-node` with a simple `villas-node`.
I'll be using the `villas` registry entry in the following sections.

## Development

You can easily setup a development shell environment for the `villas` CLI command by using the
`devShells` provided in the [`flake.nix`] using `nix develop`.
Try for example these commands in the repository root to create a new shell with
all required dependencies to build various configurations of the `villas` CLI command.

```shell
# The default creates the `#full` shell
nix develop ./packaging/nix
```

## Further Reading

- The [Nix Manual](https://nixos.org/manual/nix/stable/):
  installing and using the Nix package manager and Nix language
- The [Nixpkgs Manual](https://nixos.org/manual/nixpkgs/stable/):
  using the tools provided by Nixpkgs

## Docker/OCI Images

OCI images created using nix can be as small as the typical `FROM scratch`
images without worrying about missing dependencies or Copying things in between
`Dockerfile` stages. Copying exactly the application, it's dependencies an
nothing else can be done using only a few lines of Nix.

Here we build a docker image containing the `#villas-node` flake output:

```shell
# `docker load` reads OCI images from stdin
# `nix build --impure --expr` builds a derivation from an impure Nix expression
# `--no-link` omits creating the `result` symlink
# `--print-out-paths` prints the built image's path to stdout
docker load < $(nix build --no-link --print-out-paths --impure --expr '
  let
    villas-node = (builtins.getFlake "villas").packages.x86_64-linux.villas-node;
    pkgs = (builtins.getFlake "nixpkgs").legacyPackages.x86_64-linux;
  in
    pkgs.dockerTools.buildImage {
      name = "villas";
      tag = "nix";
      created = "now";
      copyToRoot = [villas];
      config.Cmd = ["${villas-node}/bin/villas" "node" "-h"];
    }
')
```

See <https://nixos.org/manual/nixpkgs/stable/#sec-pkgs-dockerTools>

## Customization

The [`villas.nix`] file contains the Nix "derivation" used to build the
`villas` CLI command. Check the `# customization` options at the top to find
out what optional things can be enabled.

### Building a customized `villas` CLI command

Using and customizing the villas package requires some knowledge of the Nix
expression language. And the standard `override` function used extensively in
`nixpkgs`. `override` is a Nix function provided by many packages to change
dependencies or build options. All of the inputs which can be overridden can be
found at the top of [`villas.nix`].

See <https://nixos.org/manual/nixpkgs/stable/#chap-overrides>

```shell
# `nix build` builds a derivation
# `--expr <EXPR>` specifies a Nix expression which evaluates to a "derivation"
# `--impure` allows us to use a flake without specifying the exact git revision in `ref`
# `ref` is the reference to the `villas` flake we want to use
# `pkgs` is the set of `x86_64-linux` packages provided by the flake
nix build --impure --expr '
  let
    pkgs = (builtins.getFlake "villas").packages.x86_64-linux;
  in
    pkgs.villas-node-minimal.override {
      withExtraConfig = true;
      withNodeIec60870 = true;
    }
'
```

Here we override the `#villas-node-minimal` package to include `libconfig`
configuration support and enable the `IEC61870-5-104` node.

`nix build` now builds the customized `villas` and produces a `result` symlink
in the current directory. `result` now points to a directory in the Nix store
containing the installation. You can run the `villas` binary through the
symlink.

```shell
./result/bin/villas node -h
```

### Making it persistent

Making persistent build configuration changes in a canonical way involves
writing your own flake. A flake is basically a directory containing a
`flake.nix` file. Since Nix is aware of VCS, you should create your own
`flake.nix` in a new directory outside the `villas` source tree.

Here is a basic flake to build upon:

```nix
# flake.nix
{
  description = "example of a customized villas";

  inputs = {
    villas.url = "github:VILLASframework/node?dir=packaging/nix";
  };

  outputs = { self, villas }:
  let
    villas-pkgs = villas.packages.x86_64-linux;
  in {
    packages.x86_64-linux = rec {
      default = villas-custom;
      villas-custom = villas-pkgs.villas-node-minimal.override {
        version = "custom";
        withExtraConfig = true;
        withNodeIec60870 = true;
      };
    };
  };
}
```

The attributes here are fairly simple:

- `description`: short description of the flake
- `inputs`: attribute set that specifies all dependencies of the flake
- `outputs`: a Nix function taking `self` and all fetched `inputs` producing
  the flake outputs

The `packages.x86_64-linux` output of a flake is a set of all packages
buildable by nix on a `x86_64-linux` system.

You can now build/run/install your customized `villas`. `nix` commands default
to checking the current directory (and parent directories) for a `flake.nix`
and using the `default` attribute if no `#output` is specified.

The first time you use the flake a `flake.lock` will be created pinning villas
to an exact git revision so future invocations will be exactly reproducible.

```shell
# Build custom `villas` and create the `result` symlink
nix build .

# Run custom `villas node -h`
nix run . -- node -h

# Install villas into default profile
nix profile install .

# Create a shell environment where `villas` is available in path
nix shell .

# Update the `flake.lock` if a newer `villas` is available on github
nix flake update
```

### Extending the flake

Installing the `villas` CLI command globally using `nix profile install` isn't really the
typical Nix usage. A more interesting use of Nix would be a custom Nix shell
environment containing your `villas-custom` and other tools of your choice.

Here is a more complete `flake.nix` containing `devShells` available to
`nix develop` and an OCI image.

```nix
# flake.nix
{
  description = "example of a customized villas";

  inputs = {
    # nixpkgs from the `unstable` branch
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    # VILLASnode from the official repository
    villas.url = "github:VILLASframework/node?dir=packaging/nix";

    # Overwrite the `nixpkgs` used by `villas` to our version above
    #
    # This will negate the reproducibility of the `villas` flake,
    # but allows us to deduplicate the dependencies used in our shell
    # and those in villas.
    #
    # Comment this line if an updated dependency causes a build failure
    villas.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, villas }:
  let
    villas-pkgs = villas.packages.x86_64-linux;
    pkgs = nixpkgs.packages.x86_64-linux;
  in {
    packages.x86_64-linux = rec {
      # run/build/install the `villas-custom` package by default
      default = villas-custom;

      # The customized villas package
      villas-custom = villas-pkgs.villas-node-minimal.override {
        withConfig = true;
        withNodeIec60870 = true;
      };

      # An OCI image containing `villas-custom`
      image = pkgs.dockerTools.buildImage {
        name = "villas";
        tag = "nix";
        created = "now";
        copyToRoot = [villas-custom];
        config.Cmd = ["${villas-custom}/bin/villas" "node" "-h"];
      };
    };
    devShells.x86_64-linux.default = pkgs.mkShell {
      name = "my-villas-dev-shell";
      shellHook = "exec $SHELL";
      packages = with pkgs; [
        self.packages.x86_64-linux.villas-custom
        jq
        bat
      ];
    };
  };
}
```

You can use our new features like this:

```shell
# Run your shell for villas-node
nix develop

# Load your custom OCI image into docker
docker load < $(nix build --no-link --print-out-paths .#image)
```

[`villas.nix`]: ./villas.nix
[`flake.nix`]: ./flake.nix

## License

- SPDX-FileCopyrightText: 2023 OPAL-RT Germany GmbH
- SPDX-License-Identifier: Apache-2.0
