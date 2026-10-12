# Clean standalone export

`PUBLIC_EXPORT.txt` is the literal allowlist for the self-contained project. It contains **49 files** covering standalone source, tests, configuration, scripts and documentation, including the root `LICENSE`, `.github/workflows/ci.yml`, the reproducible synthetic SVG figure and its standard-library Python generator. Private audit/delivery records, historical source and Git history, Unity assets and packages, archives, generated CSV runs, binaries and caches are excluded.

To create a fresh local directory and validate it independently:

```sh
bash scripts/export-public.sh /path/to/new/navigation-sim-export
cd /path/to/new/navigation-sim-export
dotnet restore NavigationSim.slnx --configfile NuGet.Config --locked-mode
dotnet build NavigationSim.slnx -c Release --no-restore
dotnet run --project tests/TractorNavigation.Tests -c Release --no-build
# On Bash systems, also:
bash scripts/validate.sh
```

The destination must not exist. The script copies only literal manifest files, rejects private review documents, generated paths and symlinks, and does not copy or initialize Git history, create a repository, contact GitHub or publish anything. A failed copy cleans only its own temporary directory. Review every addition to the manifest explicitly; do not replace the allowlist with a recursive repository copy.

For a distribution review, compare the exported file list with the manifest, retain `LICENSE` and algorithm attribution, repeat validation from the exported root, and inspect the staged files for sensitive or excluded material. If initializing a separate repository from an export, use fresh Git history rather than carrying historical commits into it. MIT approval for the standalone implementation does not extend to excluded historical or third-party materials.

Generate examples from the CLI instead of copying historical results. The export helper creates only a local directory; it does not publish a repository or release.
