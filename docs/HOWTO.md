# DON'T YOU DARE TO MANUALLY DEPLOY TO GH PAGES

If you're new to markdown, here's a ramp up [markdown](https://www.markdownguide.org/getting-started/)

If the existence of mkdocs.yml puzzles you, maybe you want to have a tour here [mkdocs quickstart](https://squidfunk.github.io/mkdocs-material/creating-your-site/)

Once you get here you're good to go, everything is installed in your devcontainer so no need to install anything else.

Add md files for each page, then edit [mkdocs.yml](../mkdocs.yml) accordingly.

For diagrams, drawio editor extension is enabled by devcontainer. Keep drawio files in the [diagrams](diagrams) folder.

Unfortunately mkdocs and mkdocs-material do not support natively drawio, in order to avoid tedious devcontainer maintenance (mkdocs-drawio plugin not available with brew) export your current drawio revision as png into the [images](images) folder.

***Writing documentation is important, it showcases your contribution to other people and makes their life easier understanding your work and you may even inspire them to build more things on top of that!***


## Cheat-sheet

for local mkdocs deployment, in dev container terminal (vscode) run the following in the repository root

```
mkdocs serve
```
