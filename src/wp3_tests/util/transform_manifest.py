#! /usr/bin/env python3

import uuid
import yaml
import pathlib


def unique_name():
    return uuid.uuid4().hex


class Printer:

    INDENT_WIDTH = 4

    @staticmethod
    def indent(*content: str):
        yield from map(
            lambda l: Printer.INDENT_WIDTH*' ' + l,
            content
        )

    @staticmethod
    def sections(head, *tail):
        yield from head
        for content in tail:
            yield from Printer.insert_emptyline()
            yield from content

    @staticmethod
    def insert(*content: str):
        yield from content

    @staticmethod
    def insert_emptyline():
        yield ''

    @staticmethod
    def insert_comment(comment: str):
        yield f'<!-- {comment} -->'

    @staticmethod
    def insert_launch(*content: str):
        yield '<?xml version="1.0"?>'
        yield '<launch>'
        yield from Printer.indent(*content)
        yield '</launch>'

    @staticmethod
    def insert_node(*content: str, **attrs: str):
        _attrs = {
            'name': '',
            'output': 'screen',
        }
        _attrs.update(attrs)
        assert 'pkg' in _attrs, 'Missing "pkg" attribute'
        assert 'type' in _attrs, 'Missing "type" attribute'
        if not _attrs['name']:
            fname = pathlib.Path(_attrs['type'])
            _attrs['name'] = f'{_attrs["pkg"]}__{fname.stem}__{unique_name()}'

        attr_list = ' '.join(f'{key}="{val}"' for key, val in _attrs.items())
        yield f'<node {attr_list}>'
        yield from Printer.indent(*content)
        yield '</node>'

    @staticmethod
    def insert_param(name: str, value: str):
        yield f'<param name="{name}" value="{value}"/>'

    @staticmethod
    def insert_rosparam(**kwargs):
        content = [f'{key}: {val}' for key, val in kwargs.items()]

        yield '<rosparam>'
        yield from Printer.indent(*content)
        yield '</rosparam>'

    @staticmethod
    def insert_include(*content: str, file: str):
        yield f'<include file="{file}">'
        yield from Printer.indent(*content)
        yield '</include>'

    @staticmethod
    def insert_arg(*, name: str, value: str):
        yield f'<arg name="{name}" value="{value}"/>'


class Transform:

    HOST = 'wss://intelligent-cluster-manager.herokuapp.com/signal'
    START_DELAY = '5'

    master: str
    clients: dict
    counter: int
    variables: dict

    def __init__(self, fn):

        self.master = ''
        self.clients = {}
        self.counter = 0
        self.variables = {}

        master = 'master'
        topics = []

        with open(fn) as f:
            d = yaml.load(f, yaml.Loader)

            self.variables = d.get('variables', {})

            master = d.get('master', master)
            topics += d.get('topics', [])

        for name in self.variables:
            assert name.startswith('$'), "Variables should start with '$'"

        self.master = self._eval(master)

        for topic in topics:
            self._add_topic(**topic)

    def _eval(self, name: str) -> str:
        return self.variables.get(name, name)

    def save(self, dir: pathlib.Path):

        for client in map(self._eval, self.clients):
            content = self._generate_content(client)
            content = map(str.rstrip, content)
            with open(f'{dir}/{client}.launch', 'w') as f:
                f.write('\n'.join(content))

        return self

    def print(self):
        for host, connections in self.clients.items():
            print(f'[{host}]')
            for connection in connections:
                if connection['role'] == 'publisher':
                    print(f'  ({connection["id"]:02}) {connection["topic"]}: {connection["type"]} -> {connection["client"]}')
                else:
                    print(f'  ({connection["id"]:02}) {connection["topic"]}: {connection["type"]} <- {connection["client"]}')

        return self

    def _client_category(self, client: str):
        if client.endswith('device'):
            return 'device'
        if client.endswith('video'):
            return 'video'
        if client.endswith('car'):
            return 'car'
        return 'device'

    def _add_topic(self, type, srcs, snks):
        for src in srcs:
            for snk in snks:
                self._update_client_connections(
                    host=self._eval(src['client']),
                    role='publisher',
                    client=self._eval(snk['client']),
                    topic=self._eval(src['topic']),
                    type=self._eval(type),
                    delay='0',
                )
                self._update_client_connections(
                    host=self._eval(snk['client']),
                    role='subscriber',
                    client=self._eval(src['client']),
                    topic=self._eval(snk['topic']),
                    type=self._eval(type),
                    delay='0.5',
                )
                self.counter += 1

    def _update_client_connections(self, host, role, client, topic, type, delay):
        connections = self.clients.get(host, [])
        connections.append({
            'role': role,
            'client': client,
            'id': self.counter,
            'topic': topic,
            'type': type,
            'delay': delay,
        })
        self.clients[host] = connections

    def _generate_content(self, client):
        category = self._client_category(client)
        yield from Printer.insert_launch(
            *Printer.sections(
                Printer.insert(
                    *Printer.insert_comment('Meta Data'),
                    *Printer.insert_rosparam(
                        name=client,
                        master=self.master,
                        clients=f'[{", ".join(self.clients)}]',
                    ),
                ),
                Printer.insert(
                    *Printer.insert_comment('ABConnect Wrapper'),
                    *self._generate_include(self.HOST, client, category, self.START_DELAY),
                ),
                Printer.insert(
                    *Printer.insert_comment('ABConnect Connectors'),
                    *self._generate_connections(self.clients[client]),
                ),
            ),
        )

    def _generate_include(self, host, name, category, start_delay):
        # yield from Printer.insert_include(
        #     *Printer.insert_arg(name='host', value=host),
        #     *Printer.insert_arg(name='name', value=name),
        #     *Printer.insert_arg(name='category', value=category),
        #     *Printer.insert_arg(name='start_delay', value=start_delay),
        #     file='$(find ros_abconnect)/launch/abconnect.launch',
        # )
        n = len(self.clients) - 1
        yield from Printer.insert_include(
            *Printer.insert_arg(name='n', value=str(n)),
            file='$(find wp3_tests)/launch/abconnect.launch',
        )

    def _generate_connections(self, connections):
        for conn in connections:
            yield from self._generate_remote_topic(**conn)

    def _generate_remote_topic(self, role, client, id, topic, type, delay=0):
        attrs = {
            'pkg': 'ros_abconnect',
            'type': 'add_remote_topic.py',
        }
        yield from Printer.insert_node(
            *Printer.insert_rosparam(role=role, client=client, id=id, topic=topic, type=type, delay=delay),
            **attrs,
        )


if __name__ == '__main__':

    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('filename')
    parser.add_argument('directory', nargs='?', default='launch')

    args = parser.parse_args()

    path = pathlib.Path(__file__).parent.parent / args.directory
    path.mkdir(parents=True, exist_ok=True)

    Transform(args.filename).print().save(path)


