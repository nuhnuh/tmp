Testing vim and nvim configs in a dockerized environment


# Setup and run

IMPORTANT: do not run inside a nvim terminal

    make docker-create_image-ubuntu20 # if needed

    make docker-run-ubuntu20
    sudo apt update
    sudo apt install -y \
      curl \
      git \
      neovim
    sudo apt install -y silversearcher-ag # optinal

    mkdir -p ~/.config/nvim
    cp ~/snippets-vim/vimrc.test ~/.config/nvim/init.vim  # TIP: check that file exits inside docker
    nvim

