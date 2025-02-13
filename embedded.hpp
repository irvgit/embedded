#pragma once

#if !defined(__AVR__) || !defined(__GNUC__)
    #error unsupported compiler, compile with avr-g++
#endif
#if __GNUC__ < 14
    #error out of date g++, compile with latest version
#endif
#if !defined(__OPTIMIZE__)
    #error no optimization level set, compile with -O3
#endif
#if __cplusplus < 202302L
    #error out of date c++ version, compile with -stdc++=2c
#endif
#if !defined(F_CPU)
    #error clock speed undefined, compile with -DF_CPU=N
#endif
#if !__has_include(<avr/version.h>)
    #error avr-libc not found
#endif
#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 20201UL
    #error out of date avr-libc
#endif
#if !defined(__AVR_ATmega328P__) // || with rest of mcu's after doing their metadata
    #error unsupported microcontroller (metadata has not been specified)
#endif

#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define emb_nodiscard [[nodiscard]]
#define emb_mayunused [[maybe_unused]]
#define emb_always_inline [[gnu::always_inline]]
#define emb_flatten [[gnu::flatten]]
#define emb_packed [[gnu::packed]]
#define emb_assume(...) [[assume(__VA_ARGS__)]]
#define emb_unreachable std::unreachable();
#define emb_final final
#define emb_noexcept(...) noexcept(__VA_ARGS__)
#define emb_explicit(...) explicit(__VA_ARGS__)
#define emb_delete(...) delete

#if !defined(emb_standard_library_exists)
    namespace std {
        using int8_t    = int8_t;
        using uint8_t   = uint8_t;
        using int16_t   = int16_t;
        using uint16_t  = uint16_t;
        using int32_t   = int32_t;
        using uint32_t  = uint32_t;
        using int64_t   = int64_t;
        using uint64_t  = uint64_t;
        using intmax_t  = intmax_t;
        using uintmax_t = uintmax_t;
        using size_t    = uintmax_t;
        
        //using float16_t  = _Float16;
        using float32_t  = float;
        using float64_t  = long double;
        //using float128_t = _Float128;
    
        template <bool tp_condition, typename = void>
        struct enable_if {};
        template <typename tp_type_t>
        struct enable_if<true, tp_type_t> { using type = tp_type_t; };
        template <bool tp_condition, typename tp_type_t = void>
        using enable_if_t = typename enable_if<tp_condition, tp_type_t>::type;

        template <typename tp_t>
        struct add_lvalue_reference { using type = tp_t&; };
        template <typename tp_t>
        using add_lvalue_reference_t = typename add_lvalue_reference<tp_t>::type;

        template <typename tp_t>
        struct add_rvalue_reference { using type = tp_t&&; };
        template <typename tp_t>
        using add_rvalue_reference_t = typename add_rvalue_reference<tp_t>::type;

        template <typename tp_t>
        struct add_pointer { using type = tp_t*; };
        template <typename tp_t>
        using add_pointer_t = typename add_pointer<tp_t>::type;

        template <typename tp_t>
        struct add_const            { using type = const tp_t; };
        template <typename tp_t>
        using add_const_t = typename add_const<tp_t>::type;

        template <typename tp_t>
        struct underlying_type { using type = __underlying_type(tp_t); };
        template <typename tp_t>
        using underlying_type_t = typename underlying_type<tp_t>::type;
        template <typename tp_enum_t>
        emb_nodiscard auto constexpr to_underlying(tp_enum_t p_enum) -> underlying_type_t<tp_enum_t> {
            return static_cast<underlying_type_t<tp_enum_t>>(p_enum);
        }

    template<class T, T v>
    struct integral_constant
    {
        static constexpr T value = v;
        using value_type = T;
        using type = integral_constant; // using injected-class-name
        constexpr operator value_type() const noexcept { return value; }
        constexpr value_type operator()() const noexcept { return value; } // since c++14
    };

    using true_type = std::integral_constant<bool, true>;
    using false_type = std::integral_constant<bool, false>;

    template<bool B, class T, class F>
    struct conditional { using type = T; }; 
    template<class T, class F>
    struct conditional<false, T, F> { using type = F; };
    template< bool B, class T, class F >
    using conditional_t = typename conditional<B,T,F>::type;

    template<class T, class U>
    struct is_same : std::false_type {};
    
    template<class T>
    struct is_same<T, T> : std::true_type {};

    template<class T> struct remove_cv { typedef T type; };
    template<class T> struct remove_cv<const T> { typedef T type; };
    template<class T> struct remove_cv<volatile T> { typedef T type; };
    template<class T> struct remove_cv<const volatile T> { typedef T type; };
    template<class T>
    using remove_cv_t = typename remove_cv<T>::type;
    
    template<class T> struct remove_const { typedef T type; };
    template<class T> struct remove_const<const T> { typedef T type; };
    template<class T>
    using remove_const_t = typename remove_const<T>::type;

    template<class T> struct remove_volatile { typedef T type; };
    template<class T> struct remove_volatile<volatile T> { typedef T type; };
    template<class T>
    using remove_volatile_t = typename remove_volatile<T>::type;

    template<class T> struct remove_reference { typedef T type; };
    template<class T> struct remove_reference<T&> { typedef T type; };
    template<class T> struct remove_reference<T&&> { typedef T type; };
    template< class T >
    using remove_reference_t = typename remove_reference<T>::type;

    template <class T>
    struct remove_cvref { using type = remove_reference_t<remove_volatile_t<remove_const_t<T>>>; };
    template <class T>
    using remove_cvref_t = typename remove_cvref<T>::type;

    template <typename T>
    using void_t = void;

    template<typename T>
    typename std::add_rvalue_reference<T>::type declval() noexcept {
        static_assert(false, "declval not allowed in an evaluated context");
    }

        template<class T>
        struct is_floating_point
            : std::integral_constant<
                bool,
                // Note: standard floating-point types
                std::is_same<float, typename std::remove_cv<T>::type>::value
                || std::is_same<double, typename std::remove_cv<T>::type>::value
                || std::is_same<long double, typename std::remove_cv<T>::type>::value
                //// Note: extended floating-point types (C++23, if supported)
                //|| std::is_same<std::float16_t, typename std::remove_cv<T>::type>::value
                //|| std::is_same<std::float32_t, typename std::remove_cv<T>::type>::value
                //|| std::is_same<std::float64_t, typename std::remove_cv<T>::type>::value
                //|| std::is_same<std::float128_t, typename std::remove_cv<T>::type>::value
                //|| std::is_same<std::bfloat16_t, typename std::remove_cv<T>::type>::value
            > {};
        template<class T>
        auto constexpr is_floating_point_v = is_floating_point<T>::value;

        // Note: this implementation uses C++20 facilities
        template<class T>
        struct is_integral : std::integral_constant<bool,
        requires (T t, T* p, void (*f)(T)) // T* parameter excludes reference types
        {
            reinterpret_cast<T>(t); // Exclude class types
            f(0); // Exclude enumeration types
            p + t; // Exclude everything not yet excluded but integral types
        }> {};
        template <typename tp_type_t>
        auto constexpr is_integral_v = is_integral<tp_type_t>::value;

        template <typename tp_type_t>
        struct is_arithmetic {
            auto constexpr static value = is_integral_v<tp_type_t> || is_floating_point_v<tp_type_t>;
        };
        template <typename tp_type_t>
        auto constexpr is_arithmetic_v = is_arithmetic<tp_type_t>::value;

        template <typename tp_type_t>
        concept floating_point = std::is_floating_point_v<tp_type_t>;
        template <typename tp_type_t>
        concept integral = std::is_integral_v<tp_type_t>;
        template <typename tp_type_t, typename tp_as_type_t>
        concept same_as = is_same<tp_type_t, tp_as_type_t>::value;

        struct ignore_t {
            template <typename tp_type_t>
            auto constexpr operator=(const tp_type_t&) const emb_noexcept(true) -> std::add_lvalue_reference_t<std::add_const_t<tp_type_t>> {
                return *this;
            }
        };
        auto constexpr ignore = ignore_t{};

        template <typename tp_type_t>
        struct type_identity { using type = tp_type_t; };

        /// Class template integer_sequence
        template<typename _Tp, _Tp... _Idx>
        struct integer_sequence
        {
    #if __cplusplus >= 202002L
        static_assert(is_integral_v<_Tp>);
    #endif
        typedef _Tp value_type;
        static constexpr size_t size() noexcept { return sizeof...(_Idx); }
        };
        
        template<typename _Tp, _Tp _Num>
        using make_integer_sequence
    #if __has_builtin(__make_integer_seq)
        = __make_integer_seq<integer_sequence, _Tp, _Num>;
    #else
        = integer_sequence<_Tp, __integer_pack(_Num)...>;
    #endif

        /// Alias template index_sequence
        template<size_t... _Idx>
        using index_sequence = integer_sequence<size_t, _Idx...>;

    template <size_t n, size_t i = 0, size_t... v>
    struct make_index_sequence_impl { using type = typename make_index_sequence_impl<n, i + 1, v..., i>::type; };

    template <size_t n, size_t... v>
    struct make_index_sequence_impl<n, n, v...> { using type = std::integer_sequence<size_t, v...>; };

    // Alias template for convenience
    template <size_t n>
    using make_index_sequence = typename make_index_sequence_impl<n>::type;


        template <typename tp_first_type_t, class... tp_rest_types_ts>
        struct tuple {};

    // Primary template
    template <std::size_t I, typename Tuple>
    struct tuple_element_impl;

    // Specializations for std::tuple
    template <std::size_t I, typename Head, typename... Tail>
    struct tuple_element_impl<I, std::tuple<Head, Tail...>>
        : tuple_element_impl<I - 1, std::tuple<Tail...>> {};

    // Specialization for the base case (I = 0)
    template <typename Head, typename... Tail>
    struct tuple_element_impl<0, std::tuple<Head, Tail...>> {
        using type = Head;
    };

    // Alias template for convenience
    template <std::size_t I, typename Tuple>
    using tuple_element_t = typename tuple_element_impl<I, Tuple>::type;

        template <typename tp_tuple_type_t>
        struct tuple_size;
        template <class... tp_ts>
        struct tuple_size<std::tuple<tp_ts...>> {
            auto constexpr static value = sizeof...(tp_ts);
        };
        template <typename tp_tuple_type_t>
        auto constexpr tuple_size_v = tuple_size<tp_tuple_type_t>::value;

        template <typename tp_type_t>
        auto constexpr forward(typename type_identity<tp_type_t>::type&& p_arg) emb_noexcept(true) -> tp_type_t&& {
            return static_cast<tp_type_t>(p_arg);
        }
        template<class T, class Compare>
        const constexpr T& clamp(const T& v, const T& lo, const T& hi, Compare comp) {
            return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
        }
        template <typename tp_t, uintmax_t tp_size>
        struct emb_nodiscard array emb_final {
            tp_t m_data[tp_size];
            emb_nodiscard auto constexpr operator[](const uintmax_t p_index) &        -> add_lvalue_reference_t<tp_t>              { return m_data[p_index]; }
            emb_nodiscard auto constexpr operator[](const uintmax_t p_index) &&       -> add_rvalue_reference_t<tp_t>              { return m_data[p_index]; }
            emb_nodiscard auto constexpr operator[](const uintmax_t p_index) const &  -> add_lvalue_reference_t<add_const_t<tp_t>> { return m_data[p_index]; }
            emb_nodiscard auto constexpr operator[](const uintmax_t p_index) const && -> add_rvalue_reference_t<add_const_t<tp_t>> { return m_data[p_index]; }
            emb_nodiscard auto constexpr data() const noexcept -> add_pointer_t<tp_t> { return m_data; }
            emb_nodiscard auto constexpr size() const noexcept -> uintmax_t { return tp_size; }
        };
        template <typename tp_first_t, class... tp_rest_ts>
        array(tp_first_t, tp_rest_ts...) -> array<std::enable_if_t<(... && std::same_as<tp_first_t, tp_rest_ts>), tp_first_t>, 1 + sizeof...(tp_rest_ts)>;
        [[noreturn]] inline void unreachable() {
        #if defined(_MSC_VER) && !defined(__clang__)
            __assume(false);
        #else
            __builtin_unreachable();
        #endif
        }
    };
#endif

namespace ntd {
    template <typename tp_type_t>
    struct size_of {
        auto constexpr static bytes = sizeof(tp_type_t);
        auto constexpr static bits  = bytes * 8;
    };
    template <typename tp_type_t, typename tp_as_type_t>
    concept enforce = std::same_as<std::remove_cvref_t<tp_type_t>, tp_as_type_t>;
    
    template <typename tp_type_t>
    concept arithmetic = std::integral<tp_type_t> || std::floating_point<tp_type_t>;
    template <typename tp_to_type_t, typename tp_from_type_t>
    concept static_castable_to = requires (tp_from_type_t p_from) { static_cast<tp_to_type_t>(p_from); };

    template <auto tp_first_value, auto tp_second_value>
    struct value_pair_wrap {
        auto constexpr static first  = tp_first_value;
        auto constexpr static second = tp_second_value;
    };
    template <auto tp_value>
    struct value_identity { auto constexpr static value = tp_value; };
    
    //replace with pack indexing on gcc15
    namespace detail {
        template <std::uintmax_t i, std::uintmax_t j, typename tp_first_t, class... tp_rest_ts>
        auto constexpr stogap_pack_index_impl(tp_first_t&& p_first, tp_rest_ts&&... p_rest) noexcept -> auto&& {
            if constexpr (j == i)
                return p_first;
            else return stogap_pack_index_impl<i, j + 1>(p_rest...);
        }
    };
    template <std::uintmax_t i, class... tp_types_ts>
    requires (i < sizeof...(tp_types_ts))
    auto constexpr stogap_pack_index(tp_types_ts&&... p_args) noexcept -> decltype(auto) {
        if constexpr (sizeof...(p_args) == 0)
            return;
        else return detail::stogap_pack_index_impl<i, 0>(p_args...);
    }
    template <std::uintmax_t tp_bound, typename tp_callable_t, class... tp_argument_ts>
    requires (requires (tp_callable_t p_callable, tp_argument_ts&&... p_arguments) { p_callable.template operator()<0>(std::forward<tp_argument_ts>(p_arguments)...); })
    auto constexpr match_runtime_integral_to_compiletime_constant(const std::uintmax_t p_value, tp_callable_t&& p_callable, tp_argument_ts&&... p_arguments) emb_noexcept(true) -> auto {
        return []<std::uintmax_t i>(this auto self, const std::uint8_t p_value, auto&& p_callable, auto&&... p_arguments) {
            if constexpr (i < tp_bound)
                if (i == p_value)
                    return p_callable.template operator()<i>(std::forward<tp_argument_ts>(p_arguments)...);
                else return self.template operator()<i + 1>(p_value, p_callable, std::forward<tp_argument_ts>(p_arguments)...);
        }.template operator()<0>(p_value, p_callable, std::forward<tp_argument_ts>(p_arguments)...);
    }
    template <std::integral tp_integral_t>
    emb_nodiscard auto constexpr get_bit_width_capcity(const tp_integral_t p_bit_count) emb_noexcept(true) -> std::uintmax_t {
        return (1 << p_bit_count) - 1;
    }
    template <std::uintmax_t tp_bit_count>
    auto constexpr bit_width_capcity = get_bit_width_capcity(tp_bit_count);
    
    template <std::integral tp_integral1_t, std::integral tp_integral2_t>
    emb_nodiscard auto constexpr get_bit(const tp_integral1_t p_value, const tp_integral2_t p_index) emb_noexcept(true) -> bool {
        return p_value & 1 << p_index;
    }
    template <std::integral tp_integral1_t, std::integral tp_integral2_t>
    emb_nodiscard auto constexpr set_bit(const tp_integral1_t p_value, const tp_integral2_t p_index, const bool p_state = true) emb_noexcept(true) -> tp_integral1_t {
        return p_state ? p_value | 1 << p_index : p_value & ~(1 << p_index);
    }
    template <std::integral tp_integral_t>
    emb_nodiscard auto constexpr unset_bit(const tp_integral_t p_value, const tp_integral_t p_index) emb_noexcept(true) -> tp_integral_t {
        return p_value & ~(1 << p_index);
    }
    template <std::integral tp_integral_t>
    emb_nodiscard auto constexpr bitswap(const tp_integral_t p_value) emb_noexcept(true) -> tp_integral_t {
        auto l_value = tp_integral_t{0};
        auto l_bit_count = size_of<std::remove_const_t<tp_integral_t>>::bits;
        for (auto i = std::uint8_t{0}; i < l_bit_count; ++i)
            l_value |= (static_cast<tp_integral_t>(get_bit(p_value, i)) << (l_bit_count - 1 - i));
        return l_value;
    }
    template <std::integral tp_integral1_t, std::integral tp_integral2_t>
    emb_nodiscard auto constexpr is_divisible(const tp_integral1_t p_value, const tp_integral2_t p_divisor) emb_noexcept(true) -> bool {
        return p_value % 2 == 0;
    }
    template <auto... tp_values>
    requires (ntd::is_divisible(sizeof...(tp_values), 2))
    using pairwise_values = typename decltype([]<std::size_t... tp_is>(std::index_sequence<tp_is...>) {
        return []<std::size_t... tp_js>(std::index_sequence<tp_js...>) {
            return std::type_identity<std::tuple<decltype([]<std::uintmax_t i>() { return value_pair_wrap<std::tuple_element_t<i * 2 + tp_js, std::tuple<value_identity<tp_values>...>>::value...>{}; }.template operator()<tp_is>())...>>{}; //change to pack indexing, then don't need the inner index sequence lambda either
        }(std::make_index_sequence<2>{});
    }(std::make_index_sequence<sizeof...(tp_values) / 2>{}))::type;


    template <typename tp_tuple_type_t, typename tp_callable_t, class... tp_argument_ts>
    //requires (requires (tp_callable_t p_callable, tp_argument_ts&&... p_arguments) { p_callable.template operator()<void, 0>(std::forward<tp_argument_ts>(p_arguments)...); })
    auto constexpr for_each_type_in_tuple(tp_callable_t&& p_callable, tp_argument_ts&&... p_arguments) emb_noexcept(true) -> void {
        []<std::size_t... tp_is>(std::index_sequence<tp_is...>, auto&& p_callable, auto&&... p_arguments) {
            (... , p_callable.template operator()<std::tuple_element_t<tp_is, tp_tuple_type_t>, tp_is>(std::forward<tp_argument_ts>(p_arguments)...));
        }(std::make_index_sequence<std::tuple_size_v<tp_tuple_type_t>>{}, p_callable, std::forward<tp_argument_ts>(p_arguments)...);
    }
};

namespace emb {
    inline namespace edded {
        enum class emb_nodiscard pin_mode             : std::uint8_t { input, output };
        enum class emb_nodiscard pin_type             : std::uint8_t { digital, analog };
        enum class emb_nodiscard bit_significance     : std::uint8_t { lsb, msb };
        // main registers syncronize with ports
        enum class emb_nodiscard main_register_type   : std::uint8_t {
            data_direction,
            port,
            pin_input,
        };
        // sub registers do not syncronize with ports, and are multiple
        enum class emb_nodiscard sub_register_type   : std::uint8_t {
            adc_status_a,
            adc_status_b,
            analog_comparator_status,
            digital_input_disable,
            pin_change_interrupt_mask // enable/disable individual pin change interrupts
        };
        // independant registers are are always singular
        enum class emb_nodiscard independant_register_type   : std::uint8_t {
            status,
            analog_comparator_status,
            pin_change_interrupt_control, // enable/disable interrupt groups
            pin_change_interrupt_flags    // clear interrupt groups
        };
        enum class emb_nodiscard sleep_mode : std::uint8_t {
            idle              = SLEEP_MODE_IDLE,       // cpu stopped, peripherals keep running
            adc               = SLEEP_MODE_ADC,        // cpu stopped, ADC noise reduction active
            power_down        = SLEEP_MODE_PWR_DOWN,   // entire microcontroller powered down except for external interrupts
            power_save        = SLEEP_MODE_PWR_SAVE,   // power-down mode with asynchronous timer running
            standby           = SLEEP_MODE_STANDBY,    // main oscillator running, fast wake-up
            standby_exetended = SLEEP_MODE_EXT_STANDBY // extended standby with asynchronous timer running
        };
        enum class emb_nodiscard resistor_pull        : std::uint8_t { pull_down, pull_up };
        enum class emb_nodiscard output_configuration : std::uint8_t { open_drain_pull_down, open_drain_pull_up, push_pull };
        enum class emb_nodiscard read_write_pin_flag  : std::uint8_t { none, set_mode, toggle_mode };
        enum class emb_nodiscard set_pin_mode_flag    : std::uint8_t {
            none,
            initialize_to_low,
            initialize_to_high,
        };
        enum class emb_nodiscard data_transfer_protocol : std::uint8_t {
            serial_peripheral_interface,
            two_wire_interface
        };
        enum class emb_nodiscard register_identifier : std::uint8_t {
            #if defined(PORTA)
                a,
            #endif
            #if defined(PORTB)
                b,
            #endif
            #if defined(PORTC)
                c,
            #endif
            #if defined(PORTD)
                d
            #endif
        };
        #if defined(__AVR_ATmega328P__)
            enum class emb_nodiscard timer0_prescaler : std::uint16_t { v_1 = 1, v_8 = 8,            v_64 = 64,               v_256 = 256, v_1024 = 1024 };
            enum class emb_nodiscard timer1_prescaler : std::uint16_t { v_1 = 1, v_8 = 8,            v_64 = 64,               v_256 = 256, v_1024 = 1024 };
            enum class emb_nodiscard timer2_prescaler : std::uint16_t { v_1 = 1, v_8 = 8, v_32 = 32, v_64 = 64,  v_128 = 128, v_256 = 256, v_1024 = 1024 };
        #elif defined(__AVR_ATtiny11__)
        //...//...
        #endif
        struct emb_nodiscard pin emb_final {
            register_identifier m_register_identifier = static_cast<register_identifier>(255);
            std::uint8_t        m_index               = 255;
            auto constexpr operator==(const pin& p_pin) emb_noexcept(true) -> bool {
                return m_register_identifier == p_pin.m_register_identifier && m_index == p_pin.m_index;
            }
        };
        namespace meta {
            namespace detail {
                struct pct {
                    enum emb_nodiscard : std::uint16_t {
                        nnn = 1,
                        rst = 1 << 2,
                        rxd = 1 << 3,
                        txd = 1 << 4,
                        dio = 1 << 5,
                        ain = 1 << 6,
                        itr = 1 << 7,
                        pci = 1 << 8,
                        pwm = 1 << 9,
                        adc = 1 << 10,
                        tmr = 1 << 11
                    };
                };
            };
            enum class emb_nodiscard pin_configuration_type : std::uint16_t {
                none                        = detail::pct::nnn,
                reset                       = detail::pct::rst,
                receive_data                = detail::pct::rxd,
                transmit_data               = detail::pct::txd,
                digital_input_output        = detail::pct::dio,
                analog_input                = detail::pct::ain,
                interrupt                   = detail::pct::itr,
                pin_change_interrupt        = detail::pct::pci,
                pulse_width_modulation      = detail::pct::pwm,       
                analog_to_digital_converter = detail::pct::adc,
                timer                       = detail::pct::tmr
            };

            namespace detail {
                auto constexpr pin_configuration_map = [](auto... p_args) { return std::array<std::uint16_t, sizeof...(p_args)>{static_cast<std::uint16_t>(p_args)...}; }(
                    #if defined(__AVR_ATmega328P__)
                        pct::dio | pct::pci,                       //PB0 (PCINT0/CLKO/ICP1)
                        pct::dio | pct::pci,                       //PB1 (PCINT1/OC1A)
                        pct::dio | pct::pci,                       //PB2 (PCINT2/SS/OC1B)
                        pct::dio | pct::pci | pct::pwm,            //PB3 (PCINT3/OC2A/MOSI)
                        pct::dio | pct::pci,                       //PB4 (PCINT4/MISO)
                        pct::dio | pct::pci,                       //PB5 (SCK/PCINT5)
                        pct::dio | pct::pci,                       //PB6 (PCINT6/XTAL1/TOSC1)
                        pct::dio | pct::pci,                       //PB7 (PCINT7/XTAL2/TOSC2)
                        pct::dio | pct::adc | pct::pci,            //PC0 (ADC0/PCINT8)
                        pct::dio | pct::adc | pct::pci,            //PC1 (ADC1/PCINT9)
                        pct::dio | pct::adc | pct::pci,            //PC2 (ADC2/PCINT10)
                        pct::dio | pct::adc | pct::pci,            //PC3 (ADC3/PCINT11)
                        pct::dio | pct::adc | pct::pci,            //PC4 (ADC4/SDA/PCINT12)
                        pct::dio | pct::adc | pct::pci,            //PC5 (ADC5/SCL/PCINT13)
                        pct::rst | pct::dio | pct::pci,            //PC6 (RESET/PCINT14)
                        pct::nnn,                                  //PC7
                        pct::rxd | pct::dio | pct::pci,            //PD0 (RXD/PCINT16)
                        pct::txd | pct::dio | pct::pci,            //PD1 (TXD/PCINT17)
                        pct::dio | pct::pci,                       //PD2 (INT0/PCINT18)
                        pct::dio | pct::pci | pct::pwm,            //PD3 (PCINT19/OC2B/INT1)
                        pct::dio | pct::pci | pct::pwm,            //PD4 (PCINT20/XCK/T0)
                        pct::dio | pct::pci,                       //PD5 (PCINT21/OC0B/T1)
                        pct::dio | pct::pci | pct::pwm | pct::ain, //PD6 (PCINT22/OC0A/AIN0)
                        pct::dio | pct::pci | pct::ain             //PD7 (PCINT23/AIN1)
                    #elif defined(__AVR_ATtiny11__)
                    
                    #endif
                );
            };

            auto constexpr clock_speed = F_CPU;

            template <register_identifier tp_register_identifier, std::uint8_t tp_index>
            auto constexpr is_pin = tp_index < 8 && !static_cast<bool>(detail::pin_configuration_map[std::to_underlying(tp_register_identifier) * 8 + tp_index] & std::to_underlying(pin_configuration_type::none));

            template <register_identifier tp_register_identifier, std::uint8_t tp_index, pin_configuration_type tp_pin_configuration_type>
            requires (tp_index < 8 && is_pin<tp_register_identifier, tp_index>)
            auto constexpr is_pin_configured_for = is_pin<tp_register_identifier, tp_index> && static_cast<bool>(detail::pin_configuration_map[std::to_underlying(tp_register_identifier) * 8 + tp_index] & std::to_underlying(tp_pin_configuration_type));

            auto constexpr pin_count =
                #if defined(__AVR_ATmega328P__)
                    32
                #else
                //todo: rest of mcus
                #endif
            ;
            auto constexpr programmable_io_pin_count =
                #if defined(__AVR_ATmega328P__)
                    23 //24?
                #else
                //todo: rest of mcus
                #endif
            ;
            auto constexpr timer_count =
                #if defined(TCNT7)
                    8
                #elif defined(TCNT6)
                    7
                #elif defined(TCNT5)
                    6
                #elif defined(TCNT4)
                    5
                #elif defined(TCNT3)
                    4
                #elif defined(TCNT2)
                    3
                #elif defined(TCNT1)
                    2
                #elif defined(TCNT0)
                    1
                #else
                    0
                #endif
            ;
            auto constexpr adc_channel_count =
                #if defined(ADC15D)
                    16
                #elif defined(ADC14D)
                    15
                #elif defined(ADC13D)
                    14
                #elif defined(ADC12D)
                    13
                #elif defined(ADC11D)
                    12
                #elif defined(ADC10D)
                    11
                #elif defined(ADC9D)
                    10
                #elif defined(ADC8D)
                    9
                #elif defined(ADC7D)
                    8
                #elif defined(ADC6D)
                    7
                #elif defined(ADC5D)
                    6
                #elif defined(ADC4D)
                    5
                #elif defined(ADC3D)
                    4
                #elif defined(ADC2D)
                    3
                #elif defined(ADC1D)
                    2
                #elif defined(ADC0D)
                    1
                #else
                    0
                #endif
            ;
            namespace detail {
                auto constexpr adc_channel_index_base_1_pin_map = [](auto... p_args) { return std::array<std::uint8_t, sizeof...(p_args)>{static_cast<std::uint8_t>(p_args)...}; }(
                    #if defined(__AVR_ATmega328P__)
                    0,0,0,0,0,0,0,0, //b
                    1,2,3,4,5,6,0,0, //c
                    0,0,0,0,0,0,0,0  //d
                    #else
                    //todo: rest of mcus
                    #endif
                );
            };
            template <register_identifier tp_register_identifier, std::uint8_t tp_index>
            requires (is_pin_configured_for<tp_register_identifier, tp_index, pin_configuration_type::analog_to_digital_converter>)
            auto constexpr adc_channel_index_from_pin = detail::adc_channel_index_base_1_pin_map[std::to_underlying(tp_register_identifier) * 8 + tp_index] - 1;

            namespace detail {
                auto constexpr adc_channel_pin_index_map = std::array{
                    #if defined(__AVR_ATmega328P__)
                    pin{register_identifier::c, 0}, pin{register_identifier::c, 1},
                    pin{register_identifier::c, 2}, pin{register_identifier::c, 3},
                    pin{register_identifier::c, 4}, pin{register_identifier::c, 5}
                    #else
                    //todo: rest of mcus
                    #endif
                };
            };
            template <std::uint8_t tp_index>
            requires (tp_index < adc_channel_count)
            auto constexpr adc_channel_pin_from_index = detail::adc_channel_pin_index_map[tp_index];

            namespace detail {
                auto constexpr pin_change_interrupt_base_1_index_map = std::array{
                    #if defined(__AVR_ATmega328P__)
                    1,  2,  3,  4,  5,  6,  7,  8, //b
                    9,  10, 11, 12, 13, 14, 15, 0, //c
                    17, 18, 19, 20, 21, 22, 23, 24 //c
                    #else
                    //todo: rest of mcus
                    #endif
                };
            };
            template <register_identifier tp_register_identifier, std::uint8_t tp_index>
            requires (is_pin_configured_for<tp_register_identifier, tp_index, pin_configuration_type::pin_change_interrupt>)
            auto constexpr pin_change_interrupt_index_from_pin = detail::pin_change_interrupt_base_1_index_map[std::to_underlying(tp_register_identifier) * 8 + tp_index] - 1;

            namespace detail {
                auto constexpr pin_change_interrupt_index_pin_map = std::array{
                    #if defined(__AVR_ATmega328P__)
                        pin{register_identifier::b, 0}, pin{register_identifier::b, 1},
                        pin{register_identifier::b, 2}, pin{register_identifier::b, 3},
                        pin{register_identifier::b, 4}, pin{register_identifier::b, 5},
                        pin{register_identifier::b, 6}, pin{register_identifier::b, 7},
                        pin{register_identifier::c, 0}, pin{register_identifier::c, 1},
                        pin{register_identifier::c, 2}, pin{register_identifier::c, 3},
                        pin{register_identifier::c, 4}, pin{register_identifier::c, 5},
                        pin{register_identifier::c, 6}, pin{},
                        pin{register_identifier::d, 0}, pin{register_identifier::d, 1},
                        pin{register_identifier::d, 2}, pin{register_identifier::d, 3},
                        pin{register_identifier::d, 4}, pin{register_identifier::d, 5},
                        pin{register_identifier::d, 6}, pin{register_identifier::d, 7}
                    #else
                    //todo: rest of mcus
                    #endif
                };
            };
            template <std::uint8_t tp_index>
            requires (tp_index < detail::pin_change_interrupt_index_pin_map.size() && detail::pin_change_interrupt_index_pin_map[tp_index] != pin{})
            auto constexpr pin_change_interrupt_pin_from_index = detail::pin_change_interrupt_index_pin_map[tp_index];

            namespace detail {
                auto constexpr timer_bit_width_map = std::array{
                    #if defined(__AVR_ATmega328P__)
                        8, 16, 8
                    #else
                    //todo: rest of mcus
                    #endif
                };
            };
            template <std::uint8_t tp_index>
            requires (tp_index < timer_count)
            auto constexpr timer_bit_width = detail::timer_bit_width_map[tp_index];
        };
        template <independant_register_type p_register_type>
        emb_nodiscard auto inline get_independant_register() emb_noexcept(true) -> std::add_lvalue_reference_t<volatile std::uint8_t> {
            return
                p_register_type == independant_register_type::status ? SREG :
                p_register_type == independant_register_type::analog_comparator_status ? ACSR :
                p_register_type == independant_register_type::pin_change_interrupt_control ? PCICR :
                p_register_type == independant_register_type::pin_change_interrupt_flags ? PCIFR :
                (std::unreachable(), exit(-1), SREG);
        }
        template <register_identifier tp_register_identifier, std::uint8_t tp_index, sub_register_type tp_register_type>
        requires (
            tp_index < 8 &&
            tp_register_type == sub_register_type::digital_input_disable ?
                meta::is_pin_configured_for<tp_register_identifier, tp_index,  meta::pin_configuration_type::analog_input> ||
                meta::is_pin_configured_for<tp_register_identifier, tp_index,  meta::pin_configuration_type::analog_to_digital_converter>
            :
            tp_register_type == sub_register_type::adc_status_a || tp_register_type == sub_register_type::adc_status_b ?
                meta::is_pin_configured_for<tp_register_identifier, tp_index,  meta::pin_configuration_type::analog_to_digital_converter>
            :
            tp_register_type == sub_register_type::analog_comparator_status ?
                meta::is_pin_configured_for<tp_register_identifier, tp_index,  meta::pin_configuration_type::analog_input>
            :
            tp_register_type == sub_register_type::pin_change_interrupt_mask ?
                meta::is_pin_configured_for<tp_register_identifier, tp_index,  meta::pin_configuration_type::pin_change_interrupt>
            :
            false
        )
        emb_nodiscard auto get_sub_register() emb_noexcept(true) -> std::add_lvalue_reference_t<volatile std::uint8_t> {
            return
                //p_register_type == sub_register_type::adc_status ? 000000000000000000 > 8 ? ADCSRA : ADCSRB) : // user need choose do you actually want reg a/b
                tp_register_type == sub_register_type::digital_input_disable ?
                    meta::is_pin_configured_for<tp_register_identifier, tp_index,  meta::pin_configuration_type::analog_to_digital_converter> ?
                        #if defined(DIDR2)
                            (meta::get_adc_channel_index_from_pin(p_register_identifier, p_index) <= 8 ? DIDR0 : DIDR2)
                        #else
                            DIDR0
                        #endif
                    :
                    meta::is_pin_configured_for<tp_register_identifier, tp_index,  meta::pin_configuration_type::analog_input> ?
                        DIDR1
                    :
                    (std::unreachable(), exit(-1), SREG)
                :
                tp_register_type == sub_register_type::adc_status_a ?
                    ADCSRA
                :
                tp_register_type == sub_register_type::adc_status_b ?
                    ADCSRB
                :
                tp_register_type == sub_register_type::analog_comparator_status ?
                    ACSR
                :
                tp_register_type == sub_register_type::pin_change_interrupt_mask ?
                    []() -> auto&& {
                        auto constexpr static l_quotient = meta::pin_change_interrupt_index_from_pin<tp_register_identifier, tp_index> / 8;
                        return
                            #if defined(PCMSK0)
                                l_quotient == 0 ? PCMSK0 :
                            #endif
                            #if defined(PCMSK1)
                                l_quotient == 1 ? PCMSK1 :
                            #endif
                            #if defined(PCMSK2)
                                l_quotient == 2 ? PCMSK2 :
                            #endif
                            #if defined(PCMSK3)
                                l_quotient == 3 ? PCMSK3 :
                            #endif
                            #if defined(PCMSK4)
                                l_quotient == 4 ? PCMSK4 :
                            #endif
                            #if defined(PCMSK5)
                                l_quotient == 5 ? PCMSK5 :
                            #endif
                            #if defined(PCMSK6)
                                l_quotient == 6 ? PCMSK6 :
                            #endif
                            #if defined(PCMSK7)
                                l_quotient == 7 ? PCMSK7 :
                            #endif
                            #if defined(PCMSK8)
                                l_quotient == 8 ? PCMSK8 :
                            #endif
                            #if defined(PCMSK9)
                                l_quotient == 9 ? PCMSK9 :
                            #endif
                            #if defined(PCMSK10)
                                l_quotient == 10 ? PCMSK10 :
                            #endif
                                (std::unreachable(), SREG);
                    }()
                :
                (std::unreachable(), exit(-1), SREG);
        }
        template <register_identifier tp_register_identifier, main_register_type tp_register_type>
        emb_nodiscard auto get_main_register() emb_noexcept(true) -> std::add_lvalue_reference_t<volatile std::uint8_t> {
            return                
            #if defined(DDRA)
                tp_register_identifier == register_identifier::a ?
                    tp_register_type == main_register_type::data_direction ? DDRA :
                    tp_register_type == main_register_type::pin_input ? PINA :
                    tp_register_type == main_register_type::port ? PORTA :
                    (std::unreachable(), exit(-1), SREG)
                :
            #endif
            #if defined(DDRB)
                tp_register_identifier == register_identifier::b ?
                    tp_register_type == main_register_type::data_direction ? DDRB :
                    tp_register_type == main_register_type::pin_input ? PINB :
                    tp_register_type == main_register_type::port ? PORTB :
                    (std::unreachable(), exit(-1), SREG)
                :
            #endif
            #if defined(DDRC)
                tp_register_identifier == register_identifier::c ?
                    tp_register_type == main_register_type::data_direction ? DDRC :
                    tp_register_type == main_register_type::pin_input ? PINC :
                    tp_register_type == main_register_type::port ? PORTC :
                    (std::unreachable(), exit(-1), SREG)
                :
            #endif
            #if defined(DDRD)
                tp_register_identifier == register_identifier::d ?
                    tp_register_type == main_register_type::data_direction ? DDRD :
                    tp_register_type == main_register_type::pin_input ? PIND :
                    tp_register_type == main_register_type::port ? PORTD :
                    (std::unreachable(), exit(-1), SREG)
                :
            #endif
                (std::unreachable(), exit(-1), SREG);
        }
        template <sleep_mode tp_sleep_mode>
        auto sleep() emb_noexcept(true) -> void {
            set_sleep_mode(std::to_underlying(tp_sleep_mode));
            sleep_mode();
        }
        auto inline wait_for_interrupt() emb_noexcept(true) -> void {
            sleep<sleep_mode::power_down>();
        }
        template <bool tp_state>
        auto set_global_interrupts() emb_noexcept(true) -> void {
            tp_state ? []{ sei(); }() : []{ cli(); }();
        }
        template <register_identifier tp_register_identifier, std::uint8_t tp_index, bool tp_state>
        requires (meta::is_pin_configured_for<tp_register_identifier, tp_index, meta::pin_configuration_type::pin_change_interrupt>)
        auto set_pin_change_interrupt() emb_noexcept(true) -> void {
            auto constexpr static l_index     = meta::pin_change_interrupt_index_from_pin<tp_register_identifier, tp_index>;
            auto constexpr static l_quotient  = l_index / 8;
            auto constexpr static l_remainder = l_index % 8;
            auto&& l_control_register = get_independant_register<independant_register_type::pin_change_interrupt_control>();
            auto&& l_mask_register    = get_sub_register<tp_register_identifier, tp_index, sub_register_type::pin_change_interrupt_mask>();
            l_control_register = ntd::set_bit(l_control_register, l_quotient, tp_state); 
            l_mask_register    = ntd::set_bit(l_mask_register, l_remainder, tp_state); 
        }
        template <register_identifier tp_register_identifier, std::uint8_t tp_index>
        requires (meta::is_pin_configured_for<tp_register_identifier, tp_index, meta::pin_configuration_type::pin_change_interrupt>)
        auto clear_pin_change_interrupt() emb_noexcept(true) -> void {
            auto constexpr static l_index     = meta::pin_change_interrupt_index_from_pin<tp_register_identifier, tp_index>;
            auto constexpr static l_quotient  = l_index / 8;
            auto&& l_flag_register = get_independant_register<independant_register_type::pin_change_interrupt_flags>();
            l_flag_register = ntd::set_bit(l_flag_register, l_quotient, true); 
        }
        template <register_identifier tp_register_identifier, std::uint8_t tp_index, pin_mode tp_mode, set_pin_mode_flag tp_set_pin_mode_flag = set_pin_mode_flag::none>
        requires (meta::is_pin<tp_register_identifier, tp_index>)
        auto inline set_pin_mode() emb_noexcept(true) -> void {
            auto&& l_sreg = get_independant_register<independant_register_type::status>();
            auto const l_sreg_buffer = l_sreg;
            set_global_interrupts<false>();
            auto&& l_data_direction_register = get_main_register<tp_register_identifier, main_register_type::data_direction>();
            l_data_direction_register = ntd::set_bit(l_data_direction_register, tp_index, tp_mode == pin_mode::output);
            auto&& l_port_register = get_main_register<tp_register_identifier, main_register_type::port>();
            if constexpr (tp_set_pin_mode_flag == set_pin_mode_flag::initialize_to_low)
                l_port_register = ntd::set_bit(l_port_register, tp_index, false);
            else if (tp_set_pin_mode_flag == set_pin_mode_flag::initialize_to_high)
                l_port_register = ntd::set_bit(l_port_register, tp_index, true);
            l_sreg = l_sreg_buffer;
        }
        template <const register_identifier tp_register_identifier, const std::uint8_t tp_index, std::float32_t tp_delay_us = std::float32_t{0.0}, const read_write_pin_flag tp_flag = read_write_pin_flag::none>
        requires (meta::is_pin<tp_register_identifier, tp_index>)
        emb_nodiscard auto read_pin() emb_noexcept(true) -> bool {
            if constexpr (tp_flag == read_write_pin_flag::set_mode || tp_flag == read_write_pin_flag::toggle_mode) {
                set_pin_mode<tp_register_identifier, tp_index, pin_mode::input>();
                _delay_us(tp_delay_us);
            }
            auto&& l_register = get_main_register<tp_register_identifier, main_register_type::pin_input>();
            auto l_result = ntd::get_bit(l_register, tp_index);
            _delay_us(tp_delay_us);
            if (tp_flag == read_write_pin_flag::toggle_mode) {
                set_pin_mode<tp_register_identifier, tp_index, pin_mode::output>();
                _delay_us(tp_delay_us);
            }
            return l_result;
        }
        // note: set a pin's output state using push/pull
        template <register_identifier tp_register_identifier, std::uint8_t tp_index, std::float32_t tp_delay_us = std::float32_t{0.0}, const read_write_pin_flag tp_flag = read_write_pin_flag::none>
        requires (meta::is_pin<tp_register_identifier, tp_index>)
        auto write_pin(const bool p_state) emb_noexcept(true) -> void {
            if (tp_flag == read_write_pin_flag::set_mode || tp_flag == read_write_pin_flag::toggle_mode) {
                set_pin_mode<tp_register_identifier, tp_index, pin_mode::output>();
                _delay_us(tp_delay_us);
            }
            auto&& l_register = get_main_register<tp_register_identifier, main_register_type::port>();
            l_register = ntd::set_bit(l_register, tp_index, p_state);
            _delay_us(tp_delay_us);
            if (tp_flag == read_write_pin_flag::toggle_mode) {
                set_pin_mode<tp_register_identifier, tp_index, pin_mode::input>();
                _delay_us(tp_delay_us);
            }
        }

        // note: set a pin's output state using open drain output configuration
        template <register_identifier tp_register_identifier, std::uint8_t tp_index, resistor_pull tp_resistor_pull, std::float32_t tp_delay_us = std::float32_t{0.0}>
        requires (meta::is_pin<tp_register_identifier, tp_index>)
        auto open_drain_pin(const bool p_state) -> void {
            if constexpr (tp_resistor_pull == resistor_pull::pull_up) {
                if (p_state)
                    set_pin_mode<tp_register_identifier, tp_index, pin_mode::input>();
                else set_pin_mode<tp_register_identifier, tp_index, pin_mode::output, set_pin_mode_flag::initialize_to_low>();
            }
            else {
                if (p_state)
                    set_pin_mode<tp_register_identifier, tp_index, pin_mode::output, set_pin_mode_flag::initialize_to_high>();
                else set_pin_mode<tp_register_identifier, tp_index, pin_mode::input>();
            }
            _delay_us(tp_delay_us);
        }
        // note: set a pin's output state using either push-pull or open-drain output configuration
        template <register_identifier tp_register_identifier, std::uint8_t tp_index, output_configuration tp_output_configuration, std::float32_t tp_delay_us = std::float32_t{0.0}>
        requires (meta::is_pin<tp_register_identifier, tp_index>)
        auto control_pin(const bool p_state) -> void {
            if constexpr (tp_output_configuration == output_configuration::push_pull)
                write_pin<tp_register_identifier, tp_index, tp_delay_us>(p_state);
            else open_drain_pin<tp_register_identifier, tp_index, tp_output_configuration == output_configuration::open_drain_pull_up ? resistor_pull::pull_up : resistor_pull::pull_down, tp_delay_us>(p_state);
        }
        template <register_identifier tp_register_identifier, std::uintmax_t tp_index, output_configuration tp_output_configuration = output_configuration::push_pull, std::float32_t tp_delay_us = std::float32_t{0.0}>
        requires (meta::is_pin<tp_register_identifier, tp_index>)
        class emb_nodiscard unique_pin_toggle emb_final {
        private:
            bool                m_first_state;
            bool                m_has_ownership = true;
            auto constexpr static reset_unique_pin_toggle(std::add_lvalue_reference_t<unique_pin_toggle> p_value) emb_noexcept(true) {
                p_value.m_first_state         = false;
                p_value.m_has_ownership       = false;
            }
        public:
            emb_mayunused auto operator=(unique_pin_toggle&& p_arg) emb_noexcept(true) -> std::add_lvalue_reference_t<unique_pin_toggle> {
                m_first_state         = p_arg.m_first_state;
                m_has_ownership       = p_arg.m_has_ownership;
                reset_unique_pin_toggle(p_arg);
                return *this;
            }
            unique_pin_toggle(const unique_pin_toggle&) = delete;
            unique_pin_toggle(unique_pin_toggle&& p_arg) emb_noexcept(true) : m_first_state{m_first_state}, m_has_ownership{m_has_ownership} {
                reset_unique_pin_toggle(p_arg);
            }
            emb_explicit(true) unique_pin_toggle(const bool p_first_state = true) emb_noexcept(true)
            : m_first_state{p_first_state} {
                control_pin<tp_register_identifier, tp_index, tp_output_configuration, tp_delay_us>(m_first_state);
            }
            ~unique_pin_toggle() emb_noexcept(true) {
                if (m_has_ownership)
                    control_pin<tp_register_identifier, tp_index, tp_output_configuration, tp_delay_us>(!m_first_state);
            }
        };
        class emb_nodiscard unique_global_interrupts_toggle emb_final {
        private:
            bool                m_first_state;
            bool                m_has_ownership = true;
            std::uint8_t        m_sreg;
            auto constexpr static reset_unique_global_interrupts_toggle(std::add_lvalue_reference_t<unique_global_interrupts_toggle> p_value) emb_noexcept(true) {
                p_value.m_first_state         = false;
                p_value.m_has_ownership       = false;
                p_value.m_sreg                = 0;
            }
        public:
            emb_mayunused auto operator=(unique_global_interrupts_toggle&& p_arg) emb_noexcept(true) -> std::add_lvalue_reference_t<unique_global_interrupts_toggle> {
                m_first_state         = p_arg.m_first_state;
                m_has_ownership       = p_arg.m_has_ownership;
                reset_unique_global_interrupts_toggle(p_arg);
                return *this;
            }
            unique_global_interrupts_toggle(const unique_global_interrupts_toggle&) = delete;
            unique_global_interrupts_toggle(unique_global_interrupts_toggle&& p_arg) emb_noexcept(true) : m_first_state{m_first_state}, m_has_ownership{m_has_ownership} {
                reset_unique_global_interrupts_toggle(p_arg);
            }
            emb_explicit(true) unique_global_interrupts_toggle(const bool p_first_state) emb_noexcept(true)
            : m_first_state{p_first_state}, m_sreg{get_independant_register<independant_register_type::status>()} {
                set_global_interrupts<false>();
            }
            ~unique_global_interrupts_toggle() emb_noexcept(true) {
                get_independant_register<independant_register_type::status>() = m_sreg;
            }
        };
        template <register_identifier tp_register_identifier, std::uint8_t tp_index, output_configuration tp_output_configuration = output_configuration::push_pull, std::float32_t tp_delay_us = std::float32_t{0.0}>
        requires (meta::is_pin<tp_register_identifier, tp_index>)
        auto pulse_pin(const bool p_first_state = true) emb_noexcept(true) -> void {
            unique_pin_toggle<tp_register_identifier, tp_index, tp_output_configuration, tp_delay_us>{p_first_state};
        }
        namespace detail {
            auto constexpr timer_overflow_interrupt_enable_map = std::uint8_t{0}
                #if defined(TOIE0)
                    | 1
                #endif
                #if defined(TOIE1)
                    | 1 << 1
                #endif
                #if defined(TOIE2)
                    | 1 << 2
                #endif
                #if defined(TOIE3)
                    | 1 << 3
                #endif
                #if defined(TOIE4)
                    | 1 << 4
                #endif
                #if defined(TOIE5)
                    | 1 << 5
                #endif
                #if defined(TOIE6)
                    | 1 << 6
                #endif
                #if defined(TOIE7)
                    | 1 << 7
                #endif
            ;
            template <std::uint8_t tp_timer_index>
            requires (tp_timer_index < meta::timer_count)
            auto timer_has_overflow_interrupt = ntd::get_bit(detail::timer_overflow_interrupt_enable_map, tp_timer_index);
        };

        namespace detail {
            template <std::uint8_t tp_timer_index>
            requires (tp_timer_index < meta::timer_count)
            auto get_timer_counter_control_register() emb_noexcept(true) -> auto&& {
                return
                    tp_timer_index == 0 ?
                        #if defined(TCCR0)
                            TCCR0
                        #elif defined(TCCR0B)
                            TCCR0B
                        #else
                            SREG
                        #endif
                    :
                    tp_timer_index == 1 ?
                        #if defined(TCCR1)
                            TCCR1
                        #elif defined(TCCR1B)
                            TCCR1B
                        #else
                            SREG
                        #endif
                    :
                    tp_timer_index == 2 ?
                        #if defined(TCCR2)
                            TCCR2
                        #elif defined(TCCR2B)
                            TCCR2B
                        #else
                            SREG
                        #endif
                    :
                    tp_timer_index == 3 ?
                        #if defined(TCCR3)
                            TCCR3
                        #elif defined(TCCR3B)
                            TCCR3B
                        #else
                            SREG
                        #endif
                    :
                    tp_timer_index == 4 ?
                        #if defined(TCCR4)
                            TCCR4
                        #elif defined(TCCR4B)
                            TCCR4B
                        #else
                            SREG
                        #endif
                    :
                    tp_timer_index == 5 ?
                        #if defined(TCCR5)
                            TCCR5
                        #elif defined(TCCR5B)
                            TCCR5B
                        #else
                            SREG
                        #endif
                    :
                    tp_timer_index == 6 ?
                        #if defined(TCCR6)
                            TCCR6
                        #elif defined(TCCR6B)
                            TCCR6B
                        #else
                            SREG
                        #endif
                    :
                    tp_timer_index == 7 ?
                        #if defined(TCCR7)
                            TCCR7
                        #elif defined(TCCR7B)
                            TCCR7B
                        #else
                            SREG
                        #endif
                    :
                        (std::unreachable(), SREG)
                ;
            }
            template <std::uint8_t tp_timer_index>
            requires (tp_timer_index < meta::timer_count)
            auto get_timer_count_register() emb_noexcept(true) -> std::uint8_t {
                return
                    tp_timer_index == 0 ?
                        #if defined(TCNT0)
                            TCNT0
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 1 ?
                        #if defined(TCNT1)
                            TCNT1
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 2 ?
                        #if defined(TCNT2)
                            TCNT2
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 3 ?
                        #if defined(TCNT3)
                            TCNT3
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 4 ?
                        #if defined(TCNT4)
                            TCNT4
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 5 ?
                        #if defined(TCNT5)
                            TCNT5
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 6 ?
                        #if defined(TCNT6)
                            TCNT6
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 7 ?
                        #if defined(TCNT7)
                            TCNT7
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                        (std::unreachable(), SREG)
                ;
            }
            template <std::uint8_t tp_timer_index>
            requires (tp_timer_index < meta::timer_count)
            auto get_timer_interrupt_mask_register() emb_noexcept(true) -> auto&& {
                return
                    tp_timer_index == 0 ?
                        #if defined(TIMSK0)
                            TIMSK0
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 1 ?
                        #if defined(TIMSK1)
                            TIMSK1
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 2 ?
                        #if defined(TIMSK2)
                            TIMSK2
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 3 ?
                        #if defined(TIMSK3)
                            TIMSK3
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 4 ?
                        #if defined(TIMSK4)
                            TIMSK4
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 5 ?
                        #if defined(TIMSK5)
                            TIMSK5
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 6 ?
                        #if defined(TIMSK6)
                            TIMSK6
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                    tp_timer_index == 7 ?
                        #if defined(TIMSK7)
                            TIMSK7
                        #else
                            (std::unreachable(), SREG)
                        #endif
                    :
                        (std::unreachable(), SREG)
                ;
            }
            template <std::uint8_t tp_index>
            auto seconds_counter() emb_noexcept(true) -> std::add_lvalue_reference_t<std::uint8_t volatile> {
                auto volatile static value = std::uint8_t{0};
                return value;
            };
            template <std::uint8_t tp_index>
            auto isr_timer_overflow() emb_noexcept(true) -> void {
                auto static l_overflow_count = std::uintmax_t{0};
                ++l_overflow_count;
                auto constexpr static l_timer_cycle_duration_ms = ((static_cast<std::float32_t>(ntd::bit_width_capcity<meta::timer_bit_width<tp_index>> + 1) * std::float32_t{1024}) / static_cast<std::float32_t>(meta::clock_speed));
                auto constexpr static l_timer_cycles_per_second = 1 / l_timer_cycle_duration_ms;
                if (l_overflow_count >= round(l_timer_cycles_per_second)) {
                    l_overflow_count = 0;
                    detail::seconds_counter<0>() = detail::seconds_counter<0>() + 1;
                }
            }
        };

        #if defined(emb_timer_0)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<0>(); }
        #endif
        #if defined(emb_timer_1)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<1>(); }
        #endif
        #if defined(emb_timer_2)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<2>(); }
        #endif
        #if defined(emb_timer_3)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<3>(); }
        #endif
        #if defined(emb_timer_4)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<4>(); }
        #endif
        #if defined(emb_timer_5)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<5>(); }
        #endif
        #if defined(emb_timer_6)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<6>(); }
        #endif
        #if defined(emb_timer_7)
        ISR(TIMER0_OVF_vect) { detail::isr_timer_overflow<7>(); }
        #endif

        template <std::uintmax_t tp_timer_index>
        struct prescaler_type;
        template <>
        struct prescaler_type<0> {
            using type =
                #if defined(TCNT0)
                    timer0_prescaler
                #else
                    void
                #endif
            ;
        };
        template <>
        struct prescaler_type<1> {
            using type =
                #if defined(TCNT1)
                    timer1_prescaler
                #else
                    void
                #endif
            ;
        };
        template <>
        struct prescaler_type<2> {
            using type =
                #if defined(TCNT2)
                    timer2_prescaler
                #else
                    void
                #endif
            ;
        };
        template <>
        struct prescaler_type<3> {
            using type =
                #if defined(TCNT3)
                    timer3_prescaler
                #else
                    void
                #endif
            ;
        };
        template <>
        struct prescaler_type<4> {
            using type =
                #if defined(TCNT4)
                    timer4_prescaler
                #else
                    void
                #endif
            ;
        };
        template <>
        struct prescaler_type<5> {
            using type =
                #if defined(TCNT5)
                    timer5_prescaler
                #else
                    void
                #endif
            ;
        };
        template <>
        struct prescaler_type<6> {
            using type =
                #if defined(TCNT6)
                    timer6_prescaler
                #else
                    void
                #endif
            ;
        };
        template <>
        struct prescaler_type<7> {
            using type =
                #if defined(TCNT7)
                    timer7_prescaler
                #else
                    void
                #endif
            ;
        };
        template <std::uintmax_t tp_timer_index>
        requires (tp_timer_index < meta::timer_count)
        using prescaler_type_t = typename prescaler_type<tp_timer_index>::type;

        template <std::uint8_t tp_timer_index, prescaler_type_t<tp_timer_index> tp_prescaler>
        requires (tp_timer_index < meta::timer_count)
        auto enable_timer() emb_noexcept(true) -> void {
            auto&& l_timer_counter_control_register = detail::get_timer_counter_control_register<tp_timer_index>();
            auto&& l_timer_interrupt_mask_register  = detail::get_timer_interrupt_mask_register<tp_timer_index>();
            l_timer_counter_control_register        = ntd::set_bit(l_timer_counter_control_register, CS10);
            switch (std::to_underlying(tp_prescaler)) {
                case 1:    l_timer_counter_control_register |= (1 << CS10);               break;
                case 8:    l_timer_counter_control_register |= (1 << CS11);               break;
                case 32:   l_timer_counter_control_register |= (1 << CS21) | (1 << CS20); break;
                case 64:   l_timer_counter_control_register |= (1 << CS11) | (1 << CS10); break;
                case 128:  l_timer_counter_control_register |= (1 << CS22);               break;
                case 256:  l_timer_counter_control_register |= (1 << CS12);               break;
                case 1024: l_timer_counter_control_register |= (1 << CS12) | (1 << CS10); break;
                default: emb_unreachable;
            }

            l_timer_interrupt_mask_register = ntd::set_bit(l_timer_interrupt_mask_register, tp_timer_index);
        }
        template <std::uint8_t tp_timer_index>
        requires (tp_timer_index < meta::timer_count)
        auto disable_timer() emb_noexcept(true) -> void {
            auto&& l_timer_counter_control_register = detail::get_timer_counter_control_register<tp_timer_index>();
            [] (auto... p_args) { //change to for each arg function
                auto&& l_timer_counter_control_register = detail::get_timer_counter_control_register<tp_timer_index>();
                (... , (l_timer_counter_control_register = ntd::set_bit(l_timer_counter_control_register, p_args, false)));
            }(CS10, CS11, CS12);

        }
        template <std::uint8_t tp_timer_index>
        requires (tp_timer_index < meta::timer_count)
        auto get_timer() emb_noexcept(true) -> std::uint8_t {
            return detail::seconds_counter<tp_timer_index>();
        }
        template <std::uint8_t tp_timer_index, std::integral tp_integral>
        requires (tp_timer_index < meta::timer_count)
        auto set_timer(const tp_integral p_value) emb_noexcept(true) -> void {
            detail::seconds_counter<tp_timer_index>() = p_value;
        }
        template <
            std::integral        tp_integral_t,
            register_identifier  tp_data_pin_register_identifier,
            std::uint8_t         tp_data_pin_index,
            register_identifier  tp_clock_pin_register_identifier,
            std::uint8_t         tp_clock_pin_index,
            bit_significance     tp_bit_significance,
            output_configuration tp_output_configuration = output_configuration::push_pull,
            std::float32_t       tp_delay_us = std::float32_t{0.0}
        >
        requires (
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        emb_nodiscard auto bit_bang_in() emb_noexcept(true) -> tp_integral_t {
            auto constexpr static s_bit_count = ntd::size_of<tp_integral_t>::bits;
            auto l_result = tp_integral_t{0};
            for (auto i = std::uint8_t{0}; i < s_bit_count; ++i) {
                auto l_toggle = unique_pin_toggle<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>{};
                l_result |= read_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_delay_us>() << (tp_bit_significance == bit_significance::lsb ? i : s_bit_count - 1 - i);
            }
            return l_result;
        }
        template <
            register_identifier  tp_data_pin_register_identifier,
            std::uint8_t         tp_data_pin_index,
            register_identifier  tp_clock_pin_register_identifier,
            std::uint8_t         tp_clock_pin_index,
            bit_significance     tp_bit_significance,
            output_configuration tp_output_configuration = output_configuration::push_pull,
            std::float32_t       tp_delay_us = std::float32_t{0.0},
            std::integral        tp_integral_t
        > requires (
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto bit_bang_out(tp_integral_t p_value) emb_noexcept(true) -> void {
            auto constexpr static s_bit_count = ntd::size_of<tp_integral_t>::bits;
            for (auto i = std::uint8_t{0}; i < s_bit_count; ++i) {
                auto const l_toggle = unique_pin_toggle<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>{false};
                control_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_output_configuration, tp_delay_us>(ntd::get_bit(p_value, tp_bit_significance == bit_significance::lsb ? i : (s_bit_count - 1 - i)));
            }
            control_pin<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>(false);
            control_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_output_configuration, tp_delay_us>(false);
        }
        template <
            register_identifier    tp_data_pin_register_identifier,
            std::uint8_t           tp_data_pin_index,
            register_identifier    tp_clock_pin_register_identifier,
            std::uint8_t           tp_clock_pin_index,
            register_identifier    tp_chip_select_pin_register_identifier,
            std::uint8_t           tp_chip_select_pin_index,
            data_transfer_protocol tp_data_transfer_protocol,
            output_configuration   tp_output_configuration = output_configuration::push_pull,
            std::float32_t         tp_delay_us = std::float32_t{0.0}
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::serial_peripheral_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto transfer_data_begin() -> void {
            control_pin<tp_chip_select_pin_register_identifier, tp_chip_select_pin_index, tp_output_configuration, tp_delay_us>(false);
        }
        template <
            register_identifier    tp_data_pin_register_identifier,
            std::uint8_t           tp_data_pin_index,
            register_identifier    tp_clock_pin_register_identifier,
            std::uint8_t           tp_clock_pin_index,
            data_transfer_protocol tp_data_transfer_protocol,
            output_configuration   tp_output_configuration = output_configuration::push_pull,
            std::float32_t         tp_delay_us = std::float32_t{0.0}
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::two_wire_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto transfer_data_begin() -> void {
            control_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_output_configuration, tp_delay_us>(true);
            control_pin<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>(true);
            control_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_output_configuration, tp_delay_us>(false);
            control_pin<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>(false);
        }
        template <
            register_identifier    tp_data_pin_register_identifier,
            std::uint8_t           tp_data_pin_index,
            register_identifier    tp_clock_pin_register_identifier,
            std::uint8_t           tp_clock_pin_index,
            register_identifier    tp_chip_select_register_identifier,
            std::uint8_t           tp_chip_select_pin,
            data_transfer_protocol tp_data_transfer_protocol,
            output_configuration   tp_output_configuration = output_configuration::push_pull,
            std::float32_t         tp_delay_us = std::float32_t{0.0}
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::serial_peripheral_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto transfer_data_end() -> void {
            control_pin<tp_chip_select_register_identifier, tp_chip_select_pin, tp_output_configuration, tp_delay_us>(true);
        }
        template <
            register_identifier    tp_data_pin_register_identifier,
            std::uint8_t           tp_data_pin_index,
            register_identifier    tp_clock_pin_register_identifier,
            std::uint8_t           tp_clock_pin_index,
            data_transfer_protocol tp_data_transfer_protocol,
            output_configuration   tp_output_configuration = output_configuration::push_pull,
            std::float32_t         tp_delay_us = std::float32_t{0.0}
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::two_wire_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto transfer_data_end() -> void {
            control_pin<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>(true);
            control_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_output_configuration, tp_delay_us>(true);
        }
        template <
            register_identifier    tp_data_pin_register_identifier,
            std::uint8_t           tp_data_pin_index,
            register_identifier    tp_clock_pin_register_identifier,
            std::uint8_t           tp_clock_pin_index,
            bit_significance       tp_bit_significance,
            data_transfer_protocol tp_data_transfer_protocol,
            output_configuration   tp_output_configuration = output_configuration::push_pull,
            std::float32_t         tp_delay_us = std::float32_t{0.0},
            std::integral...       tp_data_integral_ts
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::serial_peripheral_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto transfer_data_send(const tp_data_integral_ts... p_data) -> void {
            (... , bit_bang_out<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_bit_significance, tp_output_configuration, tp_delay_us>(p_data));
        }
        template <
            register_identifier           tp_data_pin_register_identifier,
            std::uint8_t                  tp_data_pin_index,
            register_identifier           tp_clock_pin_register_identifier,
            std::uint8_t                  tp_clock_pin_index,
            bit_significance              tp_bit_significance,
            data_transfer_protocol        tp_data_transfer_protocol,
            output_configuration          tp_output_configuration = output_configuration::push_pull,
            std::float32_t                tp_delay_us = std::float32_t{0.0},
            ntd::enforce<std::uint8_t>... tp_data_uint8t_ts
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::two_wire_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        emb_nodiscard auto transfer_data_send(const tp_data_uint8t_ts... p_data) -> std::uint8_t {
            return []<std::uint8_t i>(this auto self, auto... p_data) {
                if constexpr (i < sizeof...(tp_data_uint8t_ts)) {
                    bit_bang_out<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_bit_significance, tp_output_configuration, tp_delay_us>(ntd::stogap_pack_index<i>(p_data...));
                    control_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_output_configuration, tp_delay_us>(true);
                    control_pin<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>(true);
                    auto l_result = !read_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_delay_us, tp_output_configuration == output_configuration::push_pull ? read_write_pin_flag::toggle_mode : read_write_pin_flag::none>();
                    control_pin<tp_clock_pin_register_identifier, tp_clock_pin_index, tp_output_configuration, tp_delay_us>(false);
                    control_pin<tp_data_pin_register_identifier, tp_data_pin_index, tp_output_configuration, tp_delay_us>(false);
                    return !l_result ? i : self.template operator()<i + 1>(p_data...);
                }
                else return std::uint8_t{i + 1};
            }.template operator()<0>(p_data...);
        }
        template <
            register_identifier    tp_data_pin_register_identifier,
            std::uint8_t           tp_data_pin_index,
            register_identifier    tp_clock_pin_register_identifier,
            std::uint8_t           tp_clock_pin_index,
            register_identifier    tp_chip_select_register_identifier,
            std::uint8_t           tp_chip_select_pin,
            bit_significance       tp_bit_significance,
            data_transfer_protocol tp_data_transfer_protocol,
            output_configuration   tp_output_configuration = output_configuration::push_pull,
            std::float32_t         tp_delay_us = std::float32_t{0.0},
            std::integral...       tp_data_integral_ts
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::serial_peripheral_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto transfer_data(const tp_data_integral_ts... p_data) -> void {
            transfer_data_begin<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_chip_select_register_identifier, tp_chip_select_pin, tp_data_transfer_protocol, tp_output_configuration, tp_delay_us>();
            transfer_data_send<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_bit_significance, tp_data_transfer_protocol, tp_output_configuration, tp_delay_us>(p_data...);
            transfer_data_end<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_chip_select_register_identifier, tp_chip_select_pin, tp_data_transfer_protocol, tp_output_configuration, tp_delay_us>();
        }
        template <
            register_identifier           tp_data_pin_register_identifier,
            std::uint8_t                  tp_data_pin_index,
            register_identifier           tp_clock_pin_register_identifier,
            std::uint8_t                  tp_clock_pin_index,
            bit_significance              tp_bit_significance,
            data_transfer_protocol        tp_data_transfer_protocol,
            output_configuration          tp_output_configuration = output_configuration::push_pull,
            std::float32_t                tp_delay_us = std::float32_t{0.0},
            ntd::enforce<std::uint8_t>... tp_data_uint8t_ts
        >
        requires (
            tp_data_transfer_protocol == data_transfer_protocol::two_wire_interface &&
            meta::is_pin<tp_data_pin_register_identifier, tp_data_pin_index> &&
            meta::is_pin<tp_clock_pin_register_identifier, tp_clock_pin_index>
        )
        auto transfer_data(tp_data_uint8t_ts... p_data) -> std::uint8_t {
            transfer_data_begin<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_data_transfer_protocol, tp_output_configuration, tp_delay_us>();
            auto l_result = transfer_data_send<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_bit_significance, tp_data_transfer_protocol, tp_output_configuration, tp_delay_us>(p_data...);
            transfer_data_end<tp_data_pin_register_identifier, tp_data_pin_index, tp_clock_pin_register_identifier, tp_clock_pin_index, tp_data_transfer_protocol, tp_output_configuration, tp_delay_us>();
            return l_result;
        }
        namespace pins {
            namespace detail {
                template <register_identifier tp_register_identifier, std::uint8_t tp_pin>
                struct data_pin_t{};
                template <register_identifier tp_register_identifier, std::uint8_t tp_pin>
                struct clock_pin_t {};
                template <register_identifier tp_register_identifier, std::uint8_t tp_pin>
                struct chip_select_pin_t {};
                template <register_identifier tp_register_identifier, std::uint8_t tp_pin>
                struct output_pin_t{};
                template <auto...>
                struct data_pins_t{};
                template <auto...>
                struct enable_pins_t{};

                template <auto... tp_values>
                auto constexpr values_are_in_pairs_of_register_identifier_and_integral = [] {
                    using pairwise_values_t = ntd::pairwise_values<tp_values...>;
                    //TODO: need to fix this, probably need to make a more robust implementation of pairwise_values
                    //[]<std::uintmax_t... tp_is>(std::index_sequence<tp_is...>) {
                    //    //return
                    //    //    (... && (
                    //    //        std::same_as<register_identifier, std::remove_const_t<decltype(std::tuple_element_t<tp_is, pairwise_values_t>::first)>> &&
                    //    //        std::integral<std::remove_const_t<decltype(std::tuple_element_t<tp_is, pairwise_values_t>::second)>>
                    //    //    ));
                    //}(std::make_index_sequence<std::tuple_size_v<pairwise_values_t>>{});
                    return true;
                }();
            };
            template <register_identifier tp_register_identifier, std::uint8_t tp_index>
            requires (meta::is_pin<tp_register_identifier, tp_index>)
            auto constexpr data_pin = detail::data_pin_t<tp_register_identifier, tp_index>{};
            template <register_identifier tp_register_identifier, std::uint8_t tp_index>
            requires (meta::is_pin<tp_register_identifier, tp_index>)
            auto constexpr clock_pin = detail::clock_pin_t<tp_register_identifier, tp_index>{};
            template <register_identifier tp_register_identifier, std::uint8_t tp_index>
            requires (meta::is_pin<tp_register_identifier, tp_index>)
            auto constexpr chip_select_pin = detail::chip_select_pin_t<tp_register_identifier, tp_index>{};
            template <register_identifier tp_register_identifier, std::uint8_t tp_index>
            requires (meta::is_pin<tp_register_identifier, tp_index>)
            auto constexpr output_pin = detail::output_pin_t<tp_register_identifier, tp_index>{};
            template <auto... tp_register_identifiers_and_indices>
            requires (detail::values_are_in_pairs_of_register_identifier_and_integral<tp_register_identifiers_and_indices...>)
            auto constexpr data_pins = detail::data_pins_t<tp_register_identifiers_and_indices...>{};
            template <auto... tp_register_identifiers_and_indices>
            requires (detail::values_are_in_pairs_of_register_identifier_and_integral<tp_register_identifiers_and_indices...>)
            auto constexpr enable_pins = detail::enable_pins_t<tp_register_identifiers_and_indices...>{};
        };

        namespace driver {
            namespace component {
                namespace detail {
                    template <
                        register_identifier tp_output_pin_register_identifier,
                        std::uint8_t        tp_output_pin_index,
                        typename            tp_enable_pins_t,
                        typename            tp_data_pins_t
                    >
                    class emb_nodiscard multiplexer_t emb_final {
                    private:
                        auto constexpr static m_static_assertion = [] {
                            static_assert(meta::is_pin<tp_output_pin_register_identifier, tp_output_pin_index>, "invalid output pin");
                            ntd::for_each_type_in_tuple<tp_enable_pins_t>([]<typename tp_type_t, std::uintmax_t tp_index>{
                                static_assert(meta::is_pin<tp_type_t::first, tp_type_t::second>, "invalid enable pin"); // format index into error message in c++26
                            });
                            ntd::for_each_type_in_tuple<tp_data_pins_t>([]<typename tp_type_t, std::uintmax_t tp_index>{
                                static_assert(meta::is_pin<tp_type_t::first, tp_type_t::second>, "invalid data pin"); // format index into error message in c++26
                            });
                            return 0;
                        }();
                    public:
                        auto constexpr static m_chip_count  = std::tuple_size_v<tp_enable_pins_t>;
                        auto constexpr static m_bit_count   = std::tuple_size_v<tp_data_pins_t>;
                        auto constexpr static m_input_count = ntd::bit_width_capcity<m_bit_count>;
                        emb_nodiscard auto read_output() const emb_noexcept(true) -> bool {
                            return read_pin<tp_output_pin_register_identifier, tp_output_pin_index>();
                        }
                        auto set_input(const std::uintmax_t p_index) const emb_noexcept(true) -> void {//change uint8_t's to bit precist int when _BitInt/_ExtInt is available, _BitInt(16 * chip_count)
                            auto const l_chip  = p_index / m_input_count;
                            auto const l_index = p_index % m_input_count;
                            ntd::match_runtime_integral_to_compiletime_constant<m_chip_count>(l_chip, []<std::uintmax_t i>(auto l_index) {
                                using type_wrap_t = std::tuple_element_t<i, tp_enable_pins_t>;
                                auto const l_toggle = unique_pin_toggle<type_wrap_t::first, type_wrap_t::second>{false};
                                ntd::for_each_type_in_tuple<tp_data_pins_t>([]<typename tp_type_t, std::uintmax_t tp_index>(auto l_index) { write_pin<tp_type_t::first, tp_type_t::second>(ntd::get_bit(l_index, tp_index)); }, l_index);
                            }, l_index);
                        }
                        emb_nodiscard auto set_input_and_read_output(const std::uint8_t p_index) const emb_noexcept(true) -> bool {
                            set_input(p_index);
                            return read_output();
                        }
                        emb_explicit(true) multiplexer_t() emb_noexcept(true) {
                            set_pin_mode<tp_output_pin_register_identifier, tp_output_pin_index, pin_mode::input>();
                            ntd::for_each_type_in_tuple<tp_enable_pins_t>([]<typename tp_type_t, std::size_t tp_index>() { set_pin_mode<tp_type_t::first, tp_type_t::second, pin_mode::output, set_pin_mode_flag::initialize_to_low>(); });
                            ntd::for_each_type_in_tuple<tp_data_pins_t>([]<typename tp_type_t, std::size_t tp_index>() { set_pin_mode<tp_type_t::first, tp_type_t::second, pin_mode::output, set_pin_mode_flag::initialize_to_low>(); });
                            set_input(0);
                        }
                        //all_of, any_of, none_of?
                    };
                };
                struct multiplexer emb_final {
                    template <
                        register_identifier    tp_output_pin_register_identifier,
                        std::uint8_t           tp_output_pin_index,
                        auto...                tp_enable_pins_registers_and_indices,
                        auto...                tp_data_pins_registers_and_indices
                    >
                    emb_nodiscard auto static create(
                        pins::detail::output_pin_t<tp_output_pin_register_identifier, tp_output_pin_index>,
                        pins::detail::enable_pins_t<tp_enable_pins_registers_and_indices...>,
                        pins::detail::data_pins_t<tp_data_pins_registers_and_indices...>
                    ) emb_noexcept(true) {
                        return detail::multiplexer_t<
                            tp_output_pin_register_identifier,
                            tp_output_pin_index,
                            ntd::pairwise_values<tp_data_pins_registers_and_indices...>,
                            ntd::pairwise_values<tp_enable_pins_registers_and_indices...>
                        >{};
                    }
                    multiplexer() = emb_delete("multiplexer should not be instantiated, multiplexer::create() must be used to instantiate an object of type multiplexer_t.");
                };
            };

            namespace display {
                namespace detail {
                    emb_nodiscard auto inline log_n(const double p_value, const std::uintmax_t p_base) -> double {
                        return log(static_cast<double>(p_value)) / log(static_cast<double>(p_base));
                    }
                    template <std::floating_point tp_floating_point_t>
                    emb_nodiscard auto is_whole(const tp_floating_point_t p_value) -> bool {
                        return p_value == floor(p_value);
                    }
                    template <ntd::arithmetic tp_arithmetic_t>
                    emb_nodiscard auto constexpr is_negative(const tp_arithmetic_t p_value) -> bool {
                        return p_value < 0;
                    }
                    template <std::floating_point tp_floating_point_t>
                    emb_nodiscard auto floating_point_shift(tp_floating_point_t p_value, const std::uintmax_t p_base) -> std::uintmax_t {
                        return static_cast<std::uintmax_t>(round(((p_value < 1 ? double{1} : pow(p_base, static_cast<std::uintmax_t>(log_n(p_value, p_base)) + 1)) + p_value) * pow(p_base, 3)));
                    }
                    template <std::floating_point tp_floating_point_t>
                    emb_nodiscard auto count_whole_digits(tp_floating_point_t p_value, const std::uintmax_t p_base) -> std::uintmax_t {
                        return log_n(p_value, p_base) + 1;
                    }
                    template <ntd::arithmetic tp_arithmetic_t>
                    emb_nodiscard auto count_digits(tp_arithmetic_t p_value, const std::uintmax_t p_base) -> std::uintmax_t {
                        if (p_value == 0)
                            return 1;
                        if constexpr (std::is_integral_v<tp_arithmetic_t>)
                            return log_n(p_value, p_base) + 1;
                        else return count_whole_digits(p_value, p_base) + 3;
                    }
                };

                namespace detail {
                    enum class emb_nodiscard module_type : std::uint8_t {
                        max7219,
                        max7221,
                        tm1637_4,
                        tm1637_6
                    };
                    enum class emb_nodiscard allignment : std::uint8_t {
                        left,
                        right
                    };
                    enum class emb_nodiscard impedance : std::uint8_t {
                        low, high
                    };
                    enum class emb_nodiscard family : std::uint8_t {
                        mx, tm
                    };
                    emb_nodiscard auto constexpr get_family(const module_type p_module_type) emb_noexcept(true) -> family {
                        return p_module_type == module_type::max7219 || p_module_type == module_type::max7221 ? family::mx : family::tm;
                    }
                    emb_nodiscard auto constexpr get_data_transfer_protocol(const module_type p_module_type) emb_noexcept(true) -> data_transfer_protocol {
                        return p_module_type == module_type::max7219 || p_module_type == module_type::max7221 ?
                            data_transfer_protocol::serial_peripheral_interface :
                            data_transfer_protocol::two_wire_interface;
                    }
                    emb_nodiscard auto constexpr get_impedance(const module_type p_module_type) emb_noexcept(true) -> impedance {
                        return p_module_type == module_type::max7221 ? impedance::high : impedance::low;
                    }
                    emb_nodiscard auto constexpr has_chip_select(const module_type p_module_type) emb_noexcept(true) -> bool {
                        return p_module_type == module_type::max7219 || p_module_type == module_type::max7221;
                    }

                    template <
                        module_type         tp_module_type,
                        register_identifier tp_data_in_register_identifier,
                        std::uint8_t        tp_data_in_pin,
                        register_identifier tp_clock_register_identifier,
                        std::uint8_t        tp_clock_pin,
                        register_identifier tp_chip_select_register_identifier,
                        std::uint8_t        tp_chip_select_pin,
                        std::uint8_t        tp_digit_count
                    >
                    class emb_nodiscard segment7_t {
                    private:
                        enum class emb_nodiscard mx_register_code : std::uint8_t {
                            no_op         = 0X0,
                            digit_0       = 0X1,
                            digit_1       = 0X2,
                            digit_2       = 0X3,
                            digit_3       = 0X4,
                            digit_4       = 0X5,
                            digit_5       = 0X6,
                            digit_6       = 0X7,
                            digit_7       = 0X8,
                            decode_mode   = 0X9,
                            intensity     = 0XA,
                            scan_limit    = 0XB,
                            shutdown      = 0XC,
                            display_test  = 0XF
                        };
                        enum class emb_nodiscard tm_command : std::uint8_t {
                            data_set_data                  = 0b01000000,
                            data_read_key_scan             = 0b01000010,
                            data_automatic_address_adding  = 0b01000000,
                            data_fix_address               = 0b01000100,
                            data_normal_mode               = 0b01000000,
                            data_test_mode                 = 0b01100000,
                            address_digit_0                = 0b11000000,
                            address_digit_1                = 0b11000001,
                            address_digit_2                = 0b11000010,
                            address_digit_3                = 0b11000011,
                            address_digit_4                = 0b11000100,
                            address_digit_5                = 0b11000101,
                            display_control_0              = 0b10001000,
                            display_control_1              = 0b10001001,
                            display_control_2              = 0b10001010,
                            display_control_3              = 0b10001011,
                            display_control_4              = 0b10001100,
                            display_control_5              = 0b10001101,
                            display_control_6              = 0b10001110,
                            display_control_7              = 0b10001111,
                        };
                        enum class emb_nodiscard decode_mode : std::uint8_t {
                            none          = 0x00, 
                            digit_0       = 0x01,
                            digits_0_to_3 = 0x0f,
                            all           = 0xff
                        };
                        auto constexpr static m_table = std::array<std::uint8_t, 19>{
                            0b01111110, // 0
                            0b00110000, // 1
                            0b01101101, // 2
                            0b01111001, // 3
                            0b00110011, // 4
                            0b01011011, // 5
                            0b01011111, // 6
                            0b01110000, // 7
                            0b01111111, // 8
                            0b01111011, // 9
                            0b01110111, // a
                            0b00011111, // b
                            0b01001110, // c
                            0b00111101, // d
                            0b01001111, // e
                            0b01000111, // f
                            0b01111011, // g
                            0b00000001, // -
                            0b00000000  // blank
                        };
                        auto constexpr static m_tm_6digit_map = std::array<std::uint8_t, 6> {3, 4, 5, 0, 1, 2};
                        auto constexpr static m_family                              = get_family(tp_module_type);
                        auto constexpr static m_impedance                           = get_impedance(tp_module_type);
                        auto constexpr static m_data_transfer_protocol              = get_data_transfer_protocol(tp_module_type);
                        auto constexpr static m_should_power_save                   = m_impedance == impedance::high;
                        auto constexpr static m_has_chip_select                     = has_chip_select(tp_module_type);
                        auto constexpr static m_data_transfer_bit_significance      = m_family == family::mx ? bit_significance::msb : bit_significance::lsb;
                        auto constexpr static m_character_encoding_bit_significance = m_family == family::mx ? bit_significance::msb : bit_significance::lsb;

                        auto pulse_shutdown() const emb_noexcept(true) -> void {
                            set_state(true);
                            set_state(false);
                        }
                        
                        template <ntd::static_castable_to<std::uint8_t>... tp_types_ts>
                        auto static transfer_data(tp_types_ts... p_bytes) -> void {
                            if constexpr (m_family == family::mx)
                                emb::transfer_data<tp_data_in_register_identifier, tp_data_in_pin, tp_clock_register_identifier, tp_clock_pin, tp_chip_select_register_identifier, tp_chip_select_pin, m_data_transfer_bit_significance, data_transfer_protocol::serial_peripheral_interface>(static_cast<std::uint8_t>(p_bytes)...);
                            else emb::transfer_data<tp_data_in_register_identifier, tp_data_in_pin, tp_clock_register_identifier, tp_clock_pin, m_data_transfer_bit_significance, data_transfer_protocol::two_wire_interface, output_configuration::open_drain_pull_up, std::float32_t{100}>(static_cast<std::uint8_t>(p_bytes)...);
                        }
                    public:
                        auto set_state(const bool p_state) const emb_noexcept(true) -> void {
                            transfer_data(mx_register_code::shutdown, static_cast<std::uint8_t>(!p_state));
                        }
                        auto get_digit_address(const std::uint8_t p_index) const emb_noexcept(true) -> std::uint8_t {
                            if constexpr (m_family == family::mx)
                                return std::to_underlying(mx_register_code::digit_0) + p_index;
                            else return std::to_underlying(tm_command::address_digit_0) + (tp_module_type == module_type::tm1637_4 ? 0 : m_tm_6digit_map[p_index]);
                        }
                        auto get_character(const std::uint8_t p_value) const emb_noexcept(true) -> std::uint8_t {
                            return m_character_encoding_bit_significance == bit_significance::msb ? m_table[p_value] : ntd::bitswap(m_table[p_value]) >> 1;
                        }
                        auto clear() const emb_noexcept(true) -> void {
                            if constexpr (m_family == family::tm)
                                transfer_data(tm_command::data_fix_address);
                            for (auto i = std::uint8_t{0}; i < tp_digit_count; ++i)
                                transfer_data(get_digit_address(i), 0);
                            if constexpr (m_should_power_save)
                                pulse_shutdown();
                        }
                        template <ntd::arithmetic tp_arithmetic_t>
                        auto print(tp_arithmetic_t p_value, const std::uint8_t p_base = 10, const allignment p_allignment = allignment::left) const emb_noexcept(true) -> void {
                            clear();
                            auto const l_is_negative = is_negative(p_value);
                            if constexpr (m_family == family::tm)
                                transfer_data(tm_command::data_fix_address);
                            if (l_is_negative) {
                                p_value = -p_value;
                                transfer_data(get_digit_address(p_allignment == allignment::left ? tp_digit_count - 1 : count_digits(p_value, p_base) + (std::floating_point<tp_arithmetic_t> && p_value < 1)), get_character(17));
                            }
                            auto const l_offset = (p_allignment == allignment::left ? tp_digit_count - 1 - l_is_negative - (std::floating_point<tp_arithmetic_t> && p_value < 1) - (count_digits(p_value, p_base) - 1) : 0);
                            if constexpr (std::is_integral_v<tp_arithmetic_t>) {
                                if (p_value == 0)
                                    transfer_data(get_digit_address(p_allignment == allignment::left ? tp_digit_count - 1 : 0), get_character(0));
                                else for (auto i = std::uint8_t{0}; p_value != 0; p_value /= p_base, ++i)
                                    transfer_data(get_digit_address(l_offset + i), get_character(p_value % p_base));
                            }
                            else {
                                if (p_value == 0)
                                    for (auto i = std::uint8_t{0}; i < 4; ++i)
                                        transfer_data(get_digit_address((p_allignment == allignment::left ? tp_digit_count - 1 - 3 : 0) + i), get_character(0) | (i == 3 ? 0b10000000 : 0) );
                                else {
                                    auto l_value = floating_point_shift(p_value, p_base);
                                    if (p_value < 1)
                                        transfer_data(get_digit_address(p_allignment == allignment::left ? tp_digit_count - 1 - l_is_negative : count_digits(p_value, p_base)), get_character(0) | 0b10000000);
                                    for (auto i = std::uint8_t{0}; l_value != 1; l_value /= p_base, ++i)
                                        transfer_data(get_digit_address(l_offset + i), get_character(l_value % p_base) | (i == 3 ? 0b10000000 : 0));
                                }
                            }
                            if constexpr (m_should_power_save)
                                pulse_shutdown();
                        }
                        auto set_brightness(const std::uint8_t p_value) const emb_noexcept(true) -> void {
                            emb_assume(p_value <= (m_family == family::mx ? 15 : 8));
                            if constexpr (m_family == family::mx)
                                transfer_data(mx_register_code::intensity, p_value);
                            else transfer_data(std::to_underlying(tm_command::display_control_0) + p_value);
                        }
                        auto test(const bool p_state) const emb_noexcept(true) -> void {
                            transfer_data(mx_register_code::display_test, static_cast<std::uint8_t>(p_state));
                        }
                        emb_explicit(true) segment7_t() emb_noexcept(true) {
                            set_pin_mode<tp_data_in_register_identifier, tp_data_in_pin, m_family == family::mx ? pin_mode::output : pin_mode::input, set_pin_mode_flag::initialize_to_low>();
                            set_pin_mode<tp_clock_register_identifier, tp_clock_pin, m_family == family::mx ? pin_mode::output : pin_mode::input, set_pin_mode_flag::initialize_to_low>();
                            if constexpr (m_has_chip_select)
                                set_pin_mode<tp_chip_select_register_identifier, tp_chip_select_pin, pin_mode::output, set_pin_mode_flag::initialize_to_low>();
                            if constexpr (m_family == family::mx) {
                                transfer_data(mx_register_code::decode_mode, decode_mode::none);
                                transfer_data(mx_register_code::scan_limit, std::uint8_t{7});
                                test(false);
                            }
                            clear();
                            set_brightness(0);
                            if constexpr (m_family == family::mx)
                                set_state(!m_should_power_save);
                        }
                        ~segment7_t() {
                            clear();
                        }
                    };
                template <module_type tp_module_type>
                struct seven_segment {
                    template <
                        register_identifier tp_data_in_register_identifier,
                        std::uint8_t        tp_data_in_pin,
                        register_identifier tp_clock_register_identifier,
                        std::uint8_t        tp_clock_pin
                    >
                    emb_nodiscard auto static create(
                        pins::detail::data_pin_t<tp_data_in_register_identifier, tp_data_in_pin>,
                        pins::detail::clock_pin_t<tp_clock_register_identifier, tp_clock_pin>
                    ) emb_noexcept(true) {
                        return detail::segment7_t<
                            tp_module_type,
                            tp_data_in_register_identifier,
                            tp_data_in_pin,
                            tp_clock_register_identifier,
                            tp_clock_pin,
                            register_identifier{},
                            0,
                            tp_module_type == module_type::tm1637_4 ? 4 : tp_module_type == module_type::tm1637_6 ? 6 : 8
                        >{};
                    }
                    seven_segment() = emb_delete("seven_segment should not be instantiated, seven_segment::create() must be used to instantiate an object of type seven_segment_t.");
                };
                template <module_type tp_module_type>
                requires (detail::has_chip_select(tp_module_type))
                struct seven_segment<tp_module_type> {
                    template <
                        register_identifier tp_data_in_register_identifier,
                        std::uint8_t        tp_data_in_pin,
                        register_identifier tp_clock_register_identifier,
                        std::uint8_t        tp_clock_pin,
                        register_identifier tp_chip_select_register_identifier,
                        std::uint8_t        tp_chip_select_pin
                    >
                    emb_nodiscard auto static create(
                        pins::detail::data_pin_t<tp_data_in_register_identifier, tp_data_in_pin>,
                        pins::detail::clock_pin_t<tp_clock_register_identifier, tp_clock_pin>,
                        pins::detail::chip_select_pin_t<tp_chip_select_register_identifier, tp_chip_select_pin>
                    ) emb_noexcept(true) {
                        return detail::segment7_t<
                            tp_module_type,
                            tp_data_in_register_identifier,
                            tp_data_in_pin,
                            tp_clock_register_identifier,
                            tp_clock_pin,
                            tp_chip_select_register_identifier,
                            tp_chip_select_pin,
                            tp_module_type == module_type::tm1637_4 ? 4 : tp_module_type == module_type::tm1637_6 ? 6 : 8
                        >{};
                    }
                };
                };
                struct tm1637_4digit_7segment  emb_final : detail::seven_segment<detail::module_type::tm1637_4> {};
                struct tm1637_6digit_7segment  emb_final : detail::seven_segment<detail::module_type::tm1637_6> {};
                struct max7219_6digit_7segment emb_final : detail::seven_segment<detail::module_type::max7219> {};
                struct max7221_6digit_7segment emb_final : detail::seven_segment<detail::module_type::max7221> {};
            };
        };
    }
}
