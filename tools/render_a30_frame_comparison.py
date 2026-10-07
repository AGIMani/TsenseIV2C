"""Show bare/shell models at identical zero and positive thumb angles."""
from PIL import Image, ImageDraw, ImageFont
from render_a30_shell_qa import parts, render
from sync_a30_shell_frames import PACKAGE, REL, REFERENCE, reference_root


def main():
    poses = [({}, 'Zero pose'), ({'left_hand_thumb_rota_block_joint': .8}, 'rota_block = +0.8 rad'),
             ({'left_hand_thumb_rotaback_joint2': .8}, 'rotaback = +0.8 rad')]
    output = Image.new('RGB', (1800, 1520), 'white')
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 22)
    for row, (path, label, root) in enumerate([(REFERENCE, 'Bare A3.0', reference_root()),
                                               (PACKAGE / REL, 'Shell A3.0', None)]):
        for col, (pose, description) in enumerate(poses):
            objects = parts(path, pose=pose, root=root)
            picture = render(objects, (1, -.35, .25), size=(600, 720))
            output.paste(picture, (col*600, row*760+40))
            ImageDraw.Draw(output).text((col*600+15, row*760+10), f'{label} | {description}', fill='black', font=font)
    path = PACKAGE / 'qa/frame_sync_comparison.png'
    output.save(path)
    print(path)


if __name__ == '__main__':
    main()
