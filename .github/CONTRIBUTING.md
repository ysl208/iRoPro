# Contribution checklist

- [ ] Format all C++ code clang-format
- [ ] If any frontend code was modified, then the website was rebuilt with `cd frontend; polymer build --preset es6-bundled` and committed.

## Formatting C++ code
All C++ code should be formatted with clang-format.
[See the instructions here.](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/auto_code_formatting.md)

## Rebuilding the frontend
For ease of deployment, all pull requests that modify the frontend should rebuild the compiled version of the website.
```bash
cd frontend
./build.sh
```
