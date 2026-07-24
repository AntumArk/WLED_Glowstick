Import('env')

try:
    from SCons.Node import NodeList
except Exception:
    NodeList = None


if NodeList is not None and not hasattr(NodeList, 'srcnode'):
    def _node_list_srcnode(self):
        # PlatformIO's ESP-IDF builder occasionally hands SCons a NodeList here.
        # Fall back to the first concrete node so source filtering can continue.
        for node in self:
            if hasattr(node, 'srcnode'):
                return node.srcnode()
        raise AttributeError('NodeList has no srcnode')

    NodeList.srcnode = _node_list_srcnode
    print('Applied ESP-IDF NodeList compatibility patch')