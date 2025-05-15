def get_network(network_name):
    network_name = network_name.lower()
    if network_name == 'ggcnn':
        from .ggcnn import GGCNN
        return GGCNN
    elif network_name == 'ggcnn2':
        from .ggcnn2 import GGCNN2_U_Net_MHSA_Upgraded  # <- 修改为新网络类名
        return GGCNN2_U_Net_MHSA_Upgraded               # <- 修改为新网络类名
    else:
        raise NotImplementedError('Network {} is not implemented'.format(network_name))  
