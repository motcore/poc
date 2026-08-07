# web/ — pages for motcore.github.io

Self-contained HTML pages authored here and **published from the separate
`motcore.github.io` repository**. Nothing in this folder is served from `poc`.

| File | Publish as | Contents |
|------|------------|----------|
| `evolution.html` | `motcore.github.io/evolution.html` | The five design generations, what killed each one, what survived |

## Publishing

Each page is a single file — no build step, no dependencies, no external assets.
Copy it into the site repo and push:

```bash
cp web/evolution.html ../motcore.github.io/evolution.html
cd ../motcore.github.io
git add evolution.html
git commit -m "Add design evolution page"
git push
```

Then link it from the site's index page.

## Conventions

- **One file per page.** CSS and SVG inline; no external requests.
- **Same palette as the visualisers** (`cad/clutch_geometry_v5.html`): background
  `#0f172a`, panels `#1f2937`, and the per-role accents — blue for the motor
  cone, green for the output cone, pink and orange for the two ring sets, violet
  for the gear stage, yellow for apex/geometry references.
- **Superseded designs must be labelled as such** wherever they appear.

## Keeping it in sync

`evolution.html` restates [`docs/design-evolution.md`](../docs/design-evolution.md)
for a general audience. The markdown is the source of truth — when a generation
is added or a number changes, update the markdown first, then mirror it here.
