# PyBullet Typing Completions

This folder contains type stubs and typing completions for the `pybullet` Python library. By default, `pybullet` does not provide type hints or completions, which can make development less efficient in editors like Visual Studio Code. Adding these completions improves code navigation, autocompletion, and static analysis when working with `pybullet`.

Configure these variables in your `.vscode/settings.json`

```json
{
    "python.languageServer": "Default",
    "python.analysis.stubPath": "./typings"
}
```
