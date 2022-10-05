import git
import os
import datetime
import netifaces

def get_git_repo_info(path):
        res = {}

        repo = git.Repo(path)
        g = repo.git

        res['branch'] = repo.head.ref.name
        res['rev'] = g.rev_parse('HEAD')

        log = repo.head.ref.log()
        if len(log) > 0:
            last_log = log[-1]
            res['rev_time'] = datetime.datetime.fromtimestamp(last_log.time[0]).__str__()

        res['remotes'] = {}
        for r in repo.remotes:
            res['remotes'][r.name] = {'url': r.url}

        return res


def get_ip_info():
    res = {}
    for iface in netifaces.interfaces():
        res[iface] = netifaces.ifaddresses(iface)
    return res

# print(get_usb_devices())