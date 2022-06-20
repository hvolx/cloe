Cloe Launch
===========

setup-workspace
---------------

Options:

``--edit``, ``-e``
   Launch ``$EDITOR`` after creation or modification.

``--match PATTERN``, ``-m PATTERN``
   Apply profile schema to all files that match ``PATTERN``.

   - Use hash of PATTERN to identify schema::

         "*.json"                   -> .cloe/schemas/b3136a962c78ed1216a17f6c0e30b9c0.json
         "projects/prj1/**/*.yaml"  -> .cloe/schemas/projects-prj1-67da4f81.json

Example:

Initialize for VS Code project settings with the profile in question::

   cloe-launch setup-workspace -P conanfile.py -m '*.json'

1. Find project root (via nearest ``.vscode`` or ``.git`` directory)

2. If workspace settings not created yet (``.git`` directory yes, but no
   ``.vscode`` directory), then create with some default configuration:

   - Create ``.vscode/settings.json``
   - Recommend extensions for YAML in ``.vscode/settings.json``
   - Setup JSON schema matches in ``.vscode/settings.json``
   - Create sub-directory for JSON schemas: ``.cloe/schemas``

3. Run ``cloe-engine usage -j`` with selected profile to generate schema.
   Store schema in ``.cloe/schemas`` as mapped by PATTERN.

   If schema already exists, ask for confirmation to update/overwrite.
   (Or fail and require a flag to be set, ``--overwrite``, ``--no-overwrite``.)

4. Ensure that ``.vscode/settings.json`` JSON schema mapping is correct.

   - Entry exists for current match
   - No entries exist with missing schema

5. If ``--edit`` set, launch editor with ``.vscode/settings.json``.
