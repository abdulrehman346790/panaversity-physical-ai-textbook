import React from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import AuthButton from '@site/src/components/Auth/AuthButton';

export default function NavbarItemWrapper(props) {
    if (props.type === 'custom-auth-button') {
        return <AuthButton {...props} />;
    }
    return <NavbarItem {...props} />;
}
