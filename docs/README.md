# Documentation

The site is written in [Quarkdown](https://github.com/iamgio/quarkdown) 2.4.1, the version CI uses.
Run the commands from the repository root.

Live preview, rebuilt on every save:

```sh
quarkdown c docs/main.qd --out docs --out-name _preview --allow native-content --preview --watch
```

The preview gets its own folder on purpose. In preview mode Quarkdown 2.4.1 links
`<out>/script` to the `script` folder of its own install; a normal build into the same folder
then empties that link and deletes `quarkdown.min.js` from the install.

One-off build into `docs/_site/`:

```sh
quarkdown c docs/main.qd --out docs --out-name _site --allow native-content
```

`--allow native-content` is needed because `_setup.qd` adds its own CSS and script (page layout
and the hand-drawn Mermaid diagrams). Every push to `master` that touches `docs/` publishes the
site to GitHub Pages through `.github/workflows/docs.yml`.
