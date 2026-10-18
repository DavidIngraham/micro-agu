# Micro-AGU CAD

The native SolidWorks source is in [solidworks/](solidworks). Open **microAGU.SLDASM**, keeping all the adjacent parts and subassemblies together. The source snapshot is dated October 13, 2026. [manifest.json](manifest.json) records sizes and SHA-256 hashes.

Assembly references and mates are preserved in the original files. This copy has not been reopened and dependency-resolved in SolidWorks; external library references, if any, may still require their original locations. Do not treat this as a newly validated Pack and Go export.

## Rendered models and tools

The browser-ready, articulated models and their export metadata live in [viewer/assets/models](../viewer/assets/models). The [viewer tools](../viewer/tools) contain the SolidWorks export and model-processing scripts. Render assets are derived geometry; the editable design remains here.

The CAD prop file is named `APC 9x6 CW.SLDPRT`; the flight-tested prop was **9x4.5**, so the filename is not a flight hardware specification. See the [build article](../docs/micro-agu-design.md) for the actual avionics and propulsion configuration.

## Editing and versioning

Keep part filenames and the assembly directory layout intact. When modifying the design, include changed dependent parts and subassemblies in the same commit and update the checksums in `manifest.json`. SolidWorks lock and backup files are excluded from Git. Native CAD is stored directly in Git, with no LFS requirement.
